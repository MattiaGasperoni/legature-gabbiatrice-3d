#include "Config.h"
#include <fstream>
#include <stdexcept>
#include <Eigen/Dense>

Config readJSON(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) 
    {
        throw std::runtime_error("Config file '" + filename + "' not found.");
    }

    json j;
    try
    {
        file >> j;
    }
    catch (const json::parse_error& e)
    {
        throw std::runtime_error("Error parsing config.json: " + std::string(e.what()));
    }
    
    if (!j.contains("sick_settings"))
    {
        throw std::runtime_error("Missing 'sick_settings' section in config.json");
    }

    Config cfg;

    auto s = j["sick_settings"];
    cfg.sick_settings.transportProtocol = s.value("transportProtocol", "TCP");
    cfg.sick_settings.deviceIpAddr = s.value("deviceIpAddr", "192.168.7.200");
    cfg.sick_settings.receiverIp = s.value("receiverIp", "192.168.1.2");
    cfg.sick_settings.storeData = s.value("storeData", false);
    cfg.sick_settings.filePrefix = s.value("filePrefix", "");
    cfg.sick_settings.streamingPort = s.value("streamingPort", 2114);
    cfg.sick_settings.cnt = s.value("cnt", 10);
    cfg.sick_settings.showHelpAndExit = s.value("showHelpAndExit", false);

    // Settings della vista
    s = j["frame"];
    cfg.cloud_settings.scale = s.value("scale", 1.0);


    // Piano per la proiezione dei punti
    s = j["projectonPlane"];
    cfg.cloud_settings.originPlaneProjection = Eigen::Vector3d(
        s["origin"][0],
        s["origin"][1],
        s["origin"][2]
    );

    cfg.cloud_settings.normal = Eigen::Vector3d(
        s["normal"][0],
        s["normal"][1],
        s["normal"][2]
    );

    // Piani di taglio
    cfg.cloud_settings.originCutPlanes.clear();
    for (auto& pt : j["originPlane"])
        cfg.cloud_settings.originCutPlanes.push_back(Vector3d(pt[0], pt[1], pt[2]));

    cfg.cloud_settings.inclinationCutPlanes.clear();
    for (auto& inc : j["planeCutInclination"])
        cfg.cloud_settings.inclinationCutPlanes.push_back(Vector3d(inc[0], inc[1], inc[2]));


    return cfg;
}
