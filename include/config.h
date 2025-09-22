#pragma once
#include "json.hpp"
#include <string>
#include <array>
#include <vector>
#include "vector3d.h"
#include <Eigen/Dense>

// Struttura per i parametri della telecamera SICK/Visionary
struct SickSettings
{
    std::string transportProtocol;   // "TCP" o "UDP"
    std::string deviceIpAddr;        // IP del dispositivo
    std::string receiverIp;          // IP del PC ricevente (per UDP)
    bool storeData;                  // Se salvare i dati su file
    std::string filePrefix;          // Prefisso per i file di output
    std::uint16_t streamingPort;     // Porta di streaming
    unsigned cnt;                    // Numero di frame da acquisire
    std::string visionaryType;       // Tipo di Visionary ("eVisionaryTMini", ecc.)
    bool showHelpAndExit;            // Flag help
};

struct CloudSettings
{
    double scale;
    Eigen::Vector3d originPlaneProjection;
    Eigen::Vector3d normal;
    std::vector<Vector3d> originCutPlanes;
    std::vector<Vector3d> inclinationCutPlanes;
};

// Struttura per contenere tutte le configurazioni
struct Config
{
    // Parametri SICK Visionary 
    SickSettings sick_settings;

    // Parametri processing delle Point Cloud
    CloudSettings cloud_settings;
};

// Funzione per leggere la configurazione da file JSON
Config readJSON(const std::string& filename = "config.json");