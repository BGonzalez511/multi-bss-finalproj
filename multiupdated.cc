    /* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2022
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ns3/ampdu-subframe-header.h"
#include "ns3/ap-wifi-mac.h"
#include "ns3/application-container.h"
#include "ns3/boolean.h"
#include "ns3/buildings-helper.h"
#include "ns3/buildings-module.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/core-module.h"
#include "ns3/ctrl-headers.h"
#include "ns3/double.h"
#include "ns3/enum.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/frame-capture-model.h"
#include "ns3/frame-exchange-manager.h"
#include "ns3/he-configuration.h"
#include "ns3/he-phy.h"
#include "ns3/ht-phy.h"
#include "ns3/ideal-wifi-manager.h"
#include "ns3/integer.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/node-list.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-server.h"
#include "ns3/pointer.h"
#include "ns3/qos-txop.h"
#include "ns3/queue-item.h"
#include "ns3/queue-size.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simple-frame-capture-model.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/spectrum-wifi-phy.h"
#include "ns3/ssid.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/string.h"
#include "ns3/table-based-error-rate-model.h"
#include "ns3/threshold-preamble-detection-model.h"
#include "ns3/traffic-control-helper.h"
#include "ns3/traffic-control-layer.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/uinteger.h"
#include "ns3/vht-configuration.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-reception-trace-helper.h"
#include "ns3/wifi-ppdu.h"
#include "ns3/wifi-psdu.h"
#include "ns3/wifi-spectrum-signal-parameters.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <vector>
/// Avoid std::numbers::pi because it's C++20
#define PI 3.1415926535

using namespace ns3;

// Function to calculate throughput and success probability for each BSS
void CalculateThroughputAndSuccess(int numBSS, std::vector<int> successes, int payloadSize, double simulationTime) {
    for (int bss = 0; bss < numBSS; ++bss) {
        double successProbability = static_cast<double>(successes[bss]) / (successes[bss] + 1e-9); // Avoid division by zero
        double throughput = static_cast<double>(successes[bss]) * payloadSize * 8 / simulationTime; // In bits/sec
        std::cout << "BSS " << bss + 1 << ": Throughput = " << throughput / 1e6 << " Mbps, "
                  << "Success Probability = " << successProbability << std::endl;
    }
}

NS_LOG_COMPONENT_DEFINE("tgax-calibration");

WifiPhyReceptionTraceHelper wifiStats;

// Command Line Arguments
uint32_t packetSize = 1500;           ///< packet size used for the simulation (bytes)
double edThreshold = -62;             ///< Energy Detect Threshold for all secondary channels (dBm)
uint32_t seedNumber = 1;              ///< Seed number for simulation
std::string appType("constant");      ///< Application type
std::string propagationModel = "log"; ///< Propagation Loss Model to use

///< apartments; apartment-random places nodes randomly within square
///< apartment; circle-random places nodes randomly within circle
uint8_t distanceAps = 2.5; ///< Apartment size. It is the same as saying the distance between APs
double radius =
    1.1; ///< The distance in meters between the AP and the STAs. It is possible depending
         ///< on the topology that this is the max radius for randomly placed STAs
double ccaSensitivity = -82;
double duration = 1;            ///< Duration of simulation currently total_duration = (10+duration)
uint32_t networkSize = 1;       ///< Amount of STAs per AP
int apNodeCount = 1;            ///< Amount of APs
std::string standard("11ax");   ///< the 802.11 standard
std::string phyMode = "HeMcs0"; ///< Constant PHY mode. If set to "Ideal" use Ideal manager, if set
                                ///< to "auto" calculates rate to use based on distance to APs
double frequency = 5;           ///< The operating frequency band in GHz: 2.4, 5 or 6
uint16_t channelWidth = 20;     ///< The constant channel width in MHz (only for 11n/ac/ax)
uint16_t gi = 800;              ///< The guard interval in nanoseconds (800 or 400 for
                                ///< 11n/ac, 800 or 1600 or 3200 for 11 ax)
uint8_t maxMpdus = 0; ///< The maximum number of MPDUs in A-MPDUs (0 to disable MPDU aggregation)
double txPower = 50;  ///< The transmit power of all the nodes in dBm
uint16_t pktInterval = 1000;       ///< The socket packet interval in microseconds
bool enablePhyTraceHelper = false; ///< Choose wether to use the wifi-phy PhyRxbegin trace source

// Create random variable generator
Ptr<UniformRandomVariable> randomX = CreateObject<UniformRandomVariable>();
Ptr<UniformRandomVariable> randomY = CreateObject<UniformRandomVariable>();
Ptr<UniformRandomVariable> randomAngle = CreateObject<UniformRandomVariable>();
std::map<int, WifiTxVector> nodeTxVector;

NetDeviceContainer apDevices;
NetDeviceContainer staDevices;
NetDeviceContainer devices;
NodeContainer wifiNodes;
NodeContainer apNodes;
NodeContainer staNodes;

std::map<uint32_t, std::vector<double>> nodeCw;
std::map<uint32_t, std::vector<double>> nodeBackoff;
std::map<uint64_t, int> dataRateToMcs;
std::map<uint32_t, int> nodeMcs;

int appTxrec = 0;

// Function object to compute the hash of a MAC address
struct MacAddressHash
{
    std::size_t operator()(const Mac48Address& address) const;
};

std::unordered_map<Mac48Address, uint32_t, MacAddressHash> m_staMacAddressToNodeId;

uint32_t associatedStas = 0;
uint32_t deassociatedStas = 0;

std::unordered_map<uint64_t, int> bssNode; // Put node in get BSS out

std::size_t
MacAddressHash::operator()(const Mac48Address& address) const
{
    uint8_t buffer[6];
    address.CopyTo(buffer);
    std::string s(buffer, buffer + 6);
    return std::hash<std::string>{}(s);
}

uint32_t
MacAddressToNodeId(Mac48Address address)
{
    for (uint32_t i = 0; i < apDevices.GetN(); i++)
    {
        if (apDevices.Get(i)->GetAddress() == address)
        {
            return apNodes.Get(i)->GetId();
        }
    }

    auto it = m_staMacAddressToNodeId.find(address);
    if (it != m_staMacAddressToNodeId.end())
    {
        return it->second;
    }

    NS_ABORT_MSG("Found no node having MAC address " << address);
}

void
CheckStats()
{
    wifiStats.PrintAllStatistics();

    std::ofstream outFile("tx-timeline.txt");
    outFile << "Start Time,End Time,Source Node,DropReason\n";

    for (const auto& record : wifiStats.GetPpduReceptionRecord())
    {
        if (record.m_reason)
        {
            outFile << record.m_startTime.GetMilliSeconds()
                    << "," // Convert Time to a numerical format
                    << record.m_endTime.GetMilliSeconds()
                    << "," // Convert Time to a numerical format
                    << record.m_senderId << "," << record.m_reason << "\n";
        }
        else
        {
            bool allSuccess = true;
            for (const auto& status : record.m_statusPerMpdu)
            {
                if (!status)
                {
                    allSuccess = false;
                }
            }
            if (allSuccess)
            {
                outFile << record.m_startTime.GetMilliSeconds()
                        << "," // Convert Time to a numerical format
                        << record.m_endTime.GetMilliSeconds()
                        << "," // Convert Time to a numerical format
                        << record.m_senderId << ",success\n";
            }
            else
            {
                outFile << record.m_startTime.GetMilliSeconds()
                        << "," // Convert Time to a numerical format
                        << record.m_endTime.GetMilliSeconds()
                        << "," // Convert Time to a numerical format
                        << record.m_senderId << ",PayloadDecodeError\n";
            }
        }
    }
    outFile.close();
}

void
CheckAssociation()
{
    if (associatedStas < staNodes.GetN())
    {
        std::cout << "RESTARTED ASSOCIATION" << std::endl;
        for (uint32_t i = 0; i < staNodes.GetN(); i++)
        {
            Ptr<NetDevice> dev = staNodes.Get(i)->GetDevice(0);
            Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);

            Ptr<WifiPhy> dev_phy = wifi_dev->GetPhy();
            dev_phy->SetCcaSensitivityThreshold(-82);
            dev_phy->SetTxPowerStart(35);
            dev_phy->SetTxPowerEnd(35);

            Ptr<WifiMac> q_mac = wifi_dev->GetMac();

            Ptr<StaWifiMac> staMac = StaticCast<StaWifiMac>(q_mac);

            if (!(staMac->IsAssociated()))
            {
                staMac->ScanningTimeout(std::nullopt);
            }
        }
        for (uint32_t i = 0; i < apNodes.GetN(); i++)
        {
            Ptr<NetDevice> dev = apNodes.Get(i)->GetDevice(0);
            Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);

            Ptr<WifiPhy> dev_phy = wifi_dev->GetPhy();
            dev_phy->SetCcaSensitivityThreshold(-82);
            dev_phy->SetTxPowerStart(35);
            dev_phy->SetTxPowerEnd(35);
        }
        Simulator::Schedule(Seconds(1), &CheckAssociation);
    }
    else
    {
        std::cout << "associated N Sta: " << associatedStas << std::endl;
        for (uint32_t i = 0; i < staNodes.GetN(); i++)
        {
            Ptr<NetDevice> dev = staNodes.Get(i)->GetDevice(0);
            Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);

            Ptr<WifiPhy> dev_phy = wifi_dev->GetPhy();

            dev_phy->SetCcaSensitivityThreshold(ccaSensitivity);

            dev_phy->SetTxPowerStart(txPower);
            dev_phy->SetTxPowerEnd(txPower);
        }
        for (uint32_t i = 0; i < apNodes.GetN(); i++)
        {
            Ptr<NetDevice> dev = apNodes.Get(i)->GetDevice(0);
            Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);

            Ptr<WifiPhy> dev_phy = wifi_dev->GetPhy();
            // if duration longer than 67.10784 will beacon
            wifi_dev->GetMac()->SetAttribute("BeaconInterval",
                                             TimeValue(MicroSeconds(65535 * 1024)));

            dev_phy->SetCcaSensitivityThreshold(ccaSensitivity);

            dev_phy->SetTxPowerStart(txPower);
            dev_phy->SetTxPowerEnd(txPower);
        }
    }
}

void
AssociatedSta(uint16_t aid, Mac48Address addy /* addr */)
{
    associatedStas++;
    std::cout << "Node " << MacAddressToNodeId(addy)
              << " associated at T=" << Simulator::Now().GetSeconds() << std::endl;
}

void
DeAssociatedSta(uint16_t aid, Mac48Address /* addr */)
{
    deassociatedStas++;
}

std::string
AddressToString(const Address& addr)
{
    std::stringstream addressStr;
    addressStr << InetSocketAddress::ConvertFrom(addr).GetIpv4() << ":"
               << InetSocketAddress::ConvertFrom(addr).GetPort();
    return addressStr.str();
}

struct Point
{
    double x;
    double y;
};

Point
generateRandomPointInCircle(double radius, Point origin)
{
    Point result;

    double randomRadius = sqrt(radius * radius * randomAngle->GetValue(0.0, 1.0));
    while (randomRadius < 1)
    {
        randomRadius = sqrt(radius * radius * randomAngle->GetValue(0.0, 1.0));
    }
    double randomAngleInRadians = 2 * M_PI * randomAngle->GetValue(0.0, 1.0);

    result.x = origin.x + randomRadius * cos(randomAngleInRadians);
    result.y = origin.y + randomRadius * sin(randomAngleInRadians);

    return result;
}

int
main(int argc, char* argv[])
{
    int mcs = -1;

    // Disable fragmentation and RTS/CTS
    Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
                       StringValue("22000"));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("22000"));
    // Disable short retransmission failure (make retransmissions persistent)
    Config::SetDefault("ns3::WifiRemoteStationManager::MaxSlrc",
                       UintegerValue(std::numeric_limits<uint32_t>::max()));
    Config::SetDefault("ns3::WifiRemoteStationManager::MaxSsrc",
                       UintegerValue(std::numeric_limits<uint32_t>::max()));
    // Set maximum queue size to the largest value and set maximum queue delay to be larger
    // than the simulation time
    Config::SetDefault("ns3::WifiMacQueue::MaxSize",
                       QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS,
                                                100))); // TODO: set to a smaller value. 100?
    Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(Seconds(20 * duration)));

    std::string topology = "disc";

    CommandLine cmd(__FILE__);
    cmd.AddValue("pktSize", "The packet size in bytes", packetSize);
    cmd.AddValue("ed", "edThreshold for all secondary channels", edThreshold);
    cmd.AddValue("rng", "The seed run number", seedNumber);
    cmd.AddValue("app",
                 "The type of application to set. (constant,bursty,bursty-trace,setup,setup-done)",
                 appType);

    cmd.AddValue("topology", "The topology to use.", topology);
    cmd.AddValue("prop", "The propagation loss model", propagationModel);
    cmd.AddValue("distanceAps", "Set the size of the box in meters", distanceAps);
    cmd.AddValue("radius", "Set the radius in meters between the AP and the STAs", radius);
    cmd.AddValue("ccaSensitivity", "The cca sensitivity (-82dBm)", ccaSensitivity);
    cmd.AddValue("duration", "Time duration for each trial in seconds", duration);
    cmd.AddValue("networkSize", "Number of stations per bss", networkSize);
    cmd.AddValue("standard", "Set the standard (11a, 11b, 11g, 11n, 11ac, 11ax)", standard);
    cmd.AddValue("apNodes", "Number of APs", apNodeCount);
    cmd.AddValue("phyMode", "Set the constant PHY mode string used to transmit frames", phyMode);
    cmd.AddValue("frequency", "Set the operating frequency band in GHz: 2.4, 5 or 6", frequency);
    cmd.AddValue("channelWidth",
                 "Set the constant channel width in MHz (only for 11n/ac/ax)",
                 channelWidth);
    cmd.AddValue("gi",
                 "Set the the guard interval in nanoseconds (800 or 400 for 11n/ac, 800 or 1600 or "
                 "3200 for 11 ax)",
                 gi);
    cmd.AddValue("maxMpdus",
                 "Set the maximum number of MPDUs in A-MPDUs (0 to disable MPDU aggregation)",
                 maxMpdus);
    cmd.AddValue("txPower", "Set the transmit power of all nodes in dBm", txPower);
    cmd.AddValue("pktInterval", "Set the socket packet interval in microseconds", pktInterval);
    cmd.AddValue("enablePhyTraceHelper", "Enable BSS Color", enablePhyTraceHelper);

    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seedNumber);
    RngSeedManager::SetRun(seedNumber);

    // Example integration within the simulation
    std::vector<int> successes = {100, 120, 110}; // Example: Replace with actual success data for each BSS
    int payloadSize = 1500; // Example payload size in bytes
    double simulationTime = 10.0; // Example simulation time in seconds
    CalculateThroughputAndSuccess(successes.size(), successes, payloadSize, simulationTime);

    // If not default get value from command line "HeMcs10"
    if ((phyMode != "OfdmRate54Mbps") && (phyMode != "auto") && (phyMode != "ideal"))
    {
        mcs = std::stoi(phyMode.substr(phyMode.find("s") + 1));
    }

    apNodes.Create(apNodeCount);
    staNodes.Create(apNodeCount * networkSize);

    WifiStandard wifiStandard;
    if (standard == "11a")
    {
        wifiStandard = WIFI_STANDARD_80211a;
        frequency = 5;
        channelWidth = 20;
    }
    else if (standard == "11b")
    {
        wifiStandard = WIFI_STANDARD_80211b;
        frequency = 2.4;
        channelWidth = 22;
    }
    else if (standard == "11g")
    {
        wifiStandard = WIFI_STANDARD_80211g;
        frequency = 2.4;
        channelWidth = 20;
    }
    else if (standard == "11n")
    {
        if (frequency == 2.4)
        {
            wifiStandard = WIFI_STANDARD_80211n;
        }
        else if (frequency == 5)
        {
            wifiStandard = WIFI_STANDARD_80211n;
        }
        else
        {
            NS_FATAL_ERROR("Unsupported frequency band " << frequency << " GHz for standard "
                                                         << standard);
        }
    }
    else if (standard == "11ac")
    {
        wifiStandard = WIFI_STANDARD_80211ac;
        frequency = 5;
    }
    else if (standard == "11ax")
    {
        if (frequency == 2.4)
        {
            wifiStandard = WIFI_STANDARD_80211ax;
        }
        else if (frequency == 5)
        {
            wifiStandard = WIFI_STANDARD_80211ax;
        }
        else if (frequency == 6)
        {
            wifiStandard = WIFI_STANDARD_80211ax;
        }
        else
        {
            NS_FATAL_ERROR("Unsupported frequency band " << frequency << " GHz for standard "
                                                         << standard);
        }
    }
    else
    {
        NS_FATAL_ERROR("Unsupported standard: " << standard);
    }

    if (appType != "setup-done")
    {
        std::string channelStr = "{0, " + std::to_string(channelWidth) + ", BAND_" +
                                 (frequency == 2.4 ? "2_4" : (frequency == 5 ? "5" : "6")) +
                                 "GHZ, 0}";
        Config::SetDefault("ns3::WifiPhy::ChannelSettings", StringValue(channelStr));
    }

    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

    if (frequency == 6)
    {
        if (propagationModel == "log")
        {
            // Reference Loss for Friss at 1 m with 6.0 GHz
            Ptr<LogDistancePropagationLossModel> lossModel =
                CreateObject<LogDistancePropagationLossModel>();
            lossModel->SetAttribute("Exponent", DoubleValue(2.0));
            lossModel->SetAttribute("ReferenceDistance", DoubleValue(1.0));
            lossModel->SetAttribute("ReferenceLoss", DoubleValue(49.013));
            spectrumChannel->AddPropagationLossModel(lossModel);
        }
        else if (propagationModel == "fixed")
        {
            Ptr<FixedRssLossModel> lossModel = CreateObject<FixedRssLossModel>();
            lossModel->SetAttribute("Rss", DoubleValue(-80));
        }
    }
    else if (frequency == 5)
    {
        if (propagationModel == "log")
        {
            // Reference Loss for Friss at 1 m with 5.15 GHz
            Ptr<LogDistancePropagationLossModel> lossModel =
                CreateObject<LogDistancePropagationLossModel>();
            lossModel->SetAttribute("Exponent", DoubleValue(3.5));
            lossModel->SetAttribute("ReferenceDistance", DoubleValue(1.0));
            lossModel->SetAttribute("ReferenceLoss", DoubleValue(50));
            spectrumChannel->AddPropagationLossModel(lossModel);
        }
        else if (propagationModel == "fixed")
        {
            Ptr<FixedRssLossModel> lossModel = CreateObject<FixedRssLossModel>();
            lossModel->SetAttribute("Rss", DoubleValue(-80));
        }
    }
    else
    {
        // Reference Loss for Friss at 1 m with 2.4 GHz
        if (propagationModel == "log")
        {
            Ptr<LogDistancePropagationLossModel> lossModel =
                CreateObject<LogDistancePropagationLossModel>();
            lossModel->SetAttribute("Exponent", DoubleValue(2.0));
            lossModel->SetAttribute("ReferenceDistance", DoubleValue(1.0));
            lossModel->SetAttribute("ReferenceLoss", DoubleValue(40.046));
        }
        else if (propagationModel == "fixed")
        {
            Ptr<FixedRssLossModel> lossModel = CreateObject<FixedRssLossModel>();
            lossModel->SetAttribute("Rss", DoubleValue(-80));
        }
    }

    WifiHelper wifi;
    wifi.SetStandard(wifiStandard);
    if (phyMode == "ideal")
    {
        wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    }
    else
    {
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                     "DataMode",
                                     StringValue(phyMode),
                                     "ControlMode",
                                     StringValue(phyMode));
    }

    SpectrumWifiPhyHelper phy;
    phy.SetErrorRateModel("ns3::TableBasedErrorRateModel");

    phy.SetChannel(spectrumChannel);
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    if (appType != "setup-done")
    {
        phy.Set("CcaSensitivity", DoubleValue(ccaSensitivity));
        // phy.SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel",
        //                               "MinimumRssi",
        //                               DoubleValue(ccaSensitivity));
        phy.DisablePreambleDetectionModel();

        phy.Set("TxPowerStart", DoubleValue(txPower));
        phy.Set("TxPowerEnd", DoubleValue(txPower));
        // phy.Set("RxSensitivity", DoubleValue(-300));
    }
    uint64_t beaconInterval = 10 * 1024;

    WifiMacHelper mac;
    for (int i = 0; i < apNodeCount; ++i)
    {
        std::string ssi = "BSS-" + std::to_string(i);
        Ssid ssid = Ssid(ssi);
        bssNode[apNodes.Get(i)->GetId()] = i;
        mac.SetType("ns3::ApWifiMac",
                    "BeaconInterval",
                    TimeValue(MicroSeconds(beaconInterval)),
                    "Ssid",
                    SsidValue(ssid));

        NetDeviceContainer tmp = wifi.Install(phy, mac, apNodes.Get(i));

        apDevices.Add(tmp.Get(0));
        devices.Add(tmp.Get(0));
        wifiNodes.Add(apNodes.Get(i));
        std::cout << "AP MAC: " << tmp.Get(0)->GetAddress() << "," << ssi << std::endl;
    }

    for (uint32_t i = 0; i < (apNodeCount * networkSize); ++i)
    {
        // i % apNodeCount makes it so you can give each sta of the appropriate AP the
        // correct SSID
        std::string ssi = "BSS-" + std::to_string(i % apNodeCount);
        Ssid ssid = Ssid(ssi);
        bssNode[staNodes.Get(i)->GetId()] = i % apNodeCount;
        mac.SetType("ns3::StaWifiMac",
                    "MaxMissedBeacons",
                    UintegerValue(std::numeric_limits<uint32_t>::max()),
                    "Ssid",
                    SsidValue(ssid));
        NetDeviceContainer tmp = wifi.Install(phy, mac, staNodes.Get(i));

        devices.Add(tmp.Get(0));
        staDevices.Add(tmp.Get(0));
        wifiNodes.Add(staNodes.Get(i));
        std::cout << "STA: " << i << std::endl;
        std::cout << "STA MAC: " << tmp.Get(0)->GetAddress() << "," << ssi << std::endl;
    }
    wifi.AssignStreams(devices, 0);

    // Set guard interval
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/"
                "GuardInterval",
                TimeValue(NanoSeconds(gi)));

    std::tuple<double, double, double> edThresholds{edThreshold, edThreshold, edThreshold};
    // Configure AP aggregation and ED-Thresholds
    for (int i = 0; i < apNodeCount; ++i)
    {
        Ptr<NetDevice> dev = apNodes.Get(i)->GetDevice(0);

        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);

        Ptr<HeConfiguration> heConfiguration = wifi_dev->GetHeConfiguration();

        wifi_dev->GetVhtConfiguration()->SetSecondaryCcaSensitivityThresholds(edThresholds);
        wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (packetSize + 50)));
        wifi_dev->GetMac()->SetAttribute("BK_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (packetSize + 50)));
        wifi_dev->GetMac()->SetAttribute("VO_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (packetSize + 50)));
        wifi_dev->GetMac()->SetAttribute("VI_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (packetSize + 50)));

        // count associations
        wifi_dev->GetMac()->TraceConnectWithoutContext("AssociatedSta",
                                                       MakeCallback(&AssociatedSta));
        // count Desassociations
        wifi_dev->GetMac()->TraceConnectWithoutContext("DeAssociatedSta",
                                                       MakeCallback(&DeAssociatedSta));
    }
    // Configure STA aggregation
    for (uint32_t i = 0; i < (apNodeCount * networkSize); ++i)
    {
        Ptr<NetDevice> dev = staNodes.Get(i)->GetDevice(0);

        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
        wifi_dev->GetVhtConfiguration()->SetSecondaryCcaSensitivityThresholds(edThresholds);
        Ptr<HeConfiguration> heConfiguration = wifi_dev->GetHeConfiguration();

        wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (packetSize + 50)));
        wifi_dev->GetMac()->SetAttribute("BK_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (packetSize + 50)));
        wifi_dev->GetMac()->SetAttribute("VO_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (packetSize + 50)));
        wifi_dev->GetMac()->SetAttribute("VI_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (packetSize + 50)));
    }

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    randomX->SetAttribute("Stream", IntegerValue(seedNumber));
    randomX->SetAttribute("Max", DoubleValue(distanceAps));
    randomX->SetAttribute("Min", DoubleValue(0.0));

    randomY->SetAttribute("Stream", IntegerValue(seedNumber + 1));
    randomY->SetAttribute("Max", DoubleValue(distanceAps));
    randomY->SetAttribute("Min", DoubleValue(0.0));

    randomAngle->SetAttribute("Stream", IntegerValue(seedNumber + 2));
    randomAngle->SetAttribute("Max", DoubleValue(360));
    randomAngle->SetAttribute("Min", DoubleValue(0.0));

    std::vector<Point> apPositions;
    std::ofstream outFile("points.txt");
    for (uint32_t i = 0; i < apNodes.GetN(); i++)
    {
        double x = (distanceAps / 2.0);
        double y = (distanceAps / 2.0);
        if (i == 1)
        {
            x = (distanceAps / 2.0) + (distanceAps);
        }
        if (i == 2)
        {
            x = (distanceAps / 2.0);
            y = (distanceAps / 2.0) + (distanceAps);
        }
        else if (i == 3)
        {
            x = (distanceAps / 2.0) + (distanceAps);
            y = (distanceAps / 2.0) + (distanceAps);
        }
        Vector l1(x, y, 1.5);
        positionAlloc->Add(l1);
        Point apPos;
        apPos.x = x;
        apPos.y = y;
        apPositions.push_back(apPos);
        outFile << "AP" << apNodes.Get(i)->GetId() << " " << x << "," << y << std::endl;
    }

    // Set position for STAs
    std::vector<Vector> ringPos;
    std::vector<Vector> ringPosInv;
    if (topology == "disc-random")
    {
        for (uint32_t i = 0; i < staNodes.GetN(); i++)
        {
            double currentAp = bssNode[staNodes.Get(i)->GetId()];
            Point apPos = apPositions[currentAp];
            Point staPos = generateRandomPointInCircle(radius, apPos);
            Vector l1(staPos.x, staPos.y, 1.5);
            positionAlloc->Add(l1);
            outFile << "STA" << staNodes.Get(i)->GetId() << " " << staPos.x << "," << staPos.y
                    << std::endl;
        }
    }
    else if (topology == "disc")
    {
        for (uint32_t i = 0; i < staNodes.GetN(); i++)
        {
            double currentAp = bssNode[staNodes.Get(i)->GetId()];
            Point apPos = apPositions[currentAp];

            // Calculate the angle for the current point to ensure even distribution
            double angleIncrement =
                2 * M_PI / staNodes.GetN();           // Divide the circle into equal parts
            double currentAngle = i * angleIncrement; // Current angle for this station

            // Calculate the position on the circle's perimeter using the angle
            Point staPos;
            staPos.x = apPos.x + radius * cos(currentAngle);
            staPos.y = apPos.y + radius * sin(currentAngle);

            Vector l1(staPos.x, staPos.y, 1.5);
            positionAlloc->Add(l1);
            outFile << "STA" << staNodes.Get(i)->GetId() << " " << staPos.x << "," << staPos.y
                    << std::endl;
        }
    }

    outFile << "radius " << radius << std::endl;

    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(wifiNodes);

    std::vector<WifiMode> modes;
    for (uint8_t mcs = 0; mcs < 12; mcs++)
    {
        modes.push_back(WifiMode("HeMcs" + std::to_string(mcs)));
    }

    if (appType == "constant")
    {
        PacketSocketHelper packetSocket;
        packetSocket.Install(wifiNodes);

        ApplicationContainer apps;
        Ptr<UniformRandomVariable> startTime = CreateObject<UniformRandomVariable>();

        startTime->SetAttribute("Stream", IntegerValue(0));
        startTime->SetAttribute("Min", DoubleValue(6));
        startTime->SetAttribute("Max", DoubleValue(8));

        double start = 0;
        for (int i = 0; i < apNodeCount; i++)
        {
            Ptr<WifiNetDevice> wifi_apDev = DynamicCast<WifiNetDevice>(apDevices.Get(i));
            Ptr<ApWifiMac> ap_mac = DynamicCast<ApWifiMac>(wifi_apDev->GetMac());
            Ptr<PacketSocketServer> server = CreateObject<PacketSocketServer>();

            for (uint32_t x = 0; x < staNodes.GetN(); x += apNodeCount)
            {
                Ptr<WifiNetDevice> wifi_staDev = DynamicCast<WifiNetDevice>(staDevices.Get(x + i));
                Ptr<StaWifiMac> sta_mac = DynamicCast<StaWifiMac>(wifi_staDev->GetMac());

                std::cout << "Sta: " << staNodes.Get(x + i)->GetId() << " AP: " << i << std::endl;
                PacketSocketAddress socketAddr;
                socketAddr.SetSingleDevice(staDevices.Get((x + i))->GetIfIndex());
                socketAddr.SetPhysicalAddress(apDevices.Get(i)->GetAddress());
                socketAddr.SetProtocol(1);

                Ptr<PacketSocketClient> client = CreateObject<PacketSocketClient>();
                client->SetRemote(socketAddr);

                staNodes.Get(x + i)->AddApplication(client);
                client->SetAttribute("PacketSize", UintegerValue(packetSize));
                client->SetAttribute("MaxPackets", UintegerValue(0));
                client->SetAttribute("Interval", TimeValue(Time(MicroSeconds(pktInterval))));
                start = startTime->GetValue();
                client->SetStartTime(Seconds(start));
                std::cout << "APP START: " << start << std::endl;

                server->SetLocal(socketAddr);
            }
            apNodes.Get(i)->AddApplication(server);
        }
    }

    // populate m_staMacAddressToNodeId map
    for (auto it = staDevices.Begin(); it != staDevices.End(); it++)
    {
        m_staMacAddressToNodeId[Mac48Address::ConvertFrom((*it)->GetAddress())] =
            (*it)->GetNode()->GetId();
    }

    if (phyMode != "auto")
    {
        for (size_t i = 0; i < wifiNodes.GetN(); i++)
        {
            nodeMcs[wifiNodes.Get(i)->GetId()] = mcs;
        }
    }

    if (enablePhyTraceHelper)
    {
        wifiStats.Enable(wifiNodes);
        wifiStats.Start(Seconds(10));
        wifiStats.Stop(Seconds((10) + duration));
        Simulator::Schedule(Seconds((10) + duration), &CheckStats);
    }

    Simulator::Schedule(Seconds(1.5), &CheckAssociation);

    Simulator::Stop(Seconds((10) + duration));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}


// // Function to calculate throughput and success probability for each BSS
// void CalculateThroughputAndSuccess(int numBSS, std::vector<int> successes, int payloadSize, double simulationTime) {
//     for (int bss = 0; bss < numBSS; ++bss) {
//         double successProbability = static_cast<double>(successes[bss]) / (successes[bss] + 1e-9); // Avoid division by zero
//         double throughput = static_cast<double>(successes[bss]) * payloadSize * 8 / simulationTime; // In bits/sec
//         std::cout << "BSS " << bss + 1 << ": Throughput = " << throughput / 1e6 << " Mbps, "
//                   << "Success Probability = " << successProbability << std::endl;
//     }
// }

// // Example integration within the simulation
// std::vector<int> successes = {100, 120, 110}; // Example: Replace with actual success data for each BSS
// int payloadSize = 1500; // Example payload size in bytes
// double simulationTime = 10.0; // Example simulation time in seconds
// CalculateThroughputAndSuccess(successes.size(), successes, payloadSize, simulationTime);

    
