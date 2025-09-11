// =======================================================
// Macro e definizioni
// =======================================================
#define _CRT_SECURE_NO_WARNINGS

// =======================================================
// Librerie standard C++
// =======================================================
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <cassert>
#include <algorithm>    
#include <limits>       

// =======================================================
// SDK / Librerie hardware specifiche
// =======================================================
#include <FrameGrabber.h>
#include <NetLink.h>
#include <PointCloudPlyWriter.h>
#include <PointXYZ.h>
#include <VisionaryControl.h>
#include <VisionaryType.h>

// =======================================================
// Header locali del progetto
// =======================================================
#include "BlobServerConfig.h"
#include "UdpParsing.h"
#include "exitcodes.h"
#include "Config.h"
#include "PointCloudProcessor.h"


// =======================================================
// Framework esterni (Open3D, OpenCV)
// =======================================================
// Disabilita warning di troncamento e conversione solo per Open3D
#pragma warning(push)
#pragma warning(disable : 4267) // size_t -> unsigned int
#pragma warning(disable : 4305) // double -> float
#include "open3d/Open3D.h"
#pragma warning(pop)
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>


static ExitCode 
runContinuousStreamingDemo(
    visionary::VisionaryType visionaryType,
    const std::string&       transportProtocol,
    const std::string&       ipAddress,
    const std::string&       receiverIp,
    std::uint16_t            streamingPort,
    unsigned                 numberOfFrames,
    const std::string&       filePrefix,
    bool                     storeData,
    double                   scale,
    Eigen::Vector3d          originPlaneProjecton,
    Eigen::Vector3d          normal,
    std::vector<Vector3d>    originCutPlanes,
    std::vector<Vector3d>    inclinationCutPlanes)
{
    // tag::open_control_channel[]
    using namespace visionary;

    std::shared_ptr<VisionaryControl> visionaryControl = std::make_shared<VisionaryControl>(visionaryType);

    // Connect to devices control channel
    std::cout << "[Step] Tentativo di connessione con il dispositivo a " << ipAddress << "...\n";
    if (!visionaryControl->open(ipAddress))
    {
        std::fprintf(stderr, "[Errore] Impossibile stabilire la connessione con il dispositivo.\n");
        return ExitCode::eControlCommunicationError;
    }
    std::cout << "[Step] Connessione stabilita con successo.\n";


    // end::open_control_channel[]

    //-----------------------------------------------
    // Stop image acquisition (works always, also when already stopped)
    // Further you should always stop the device before reconfiguring it
    // tag::precautionary_stop[]
    if (!visionaryControl->stopAcquisition())
    {
        std::fprintf(stderr, "Failed to stop acquisition.\n");

        return ExitCode::eControlCommunicationError;
    }

    // end::precautionary_stop[]
    // Depending on the PC we might be too fast for the device configuration
    // Just wait a short time. This should only be necessary after stop
    // (to make sure stop really propagated and you don't get a pending frame)
    // or after a configure to make sure configuration has finished
    // tag::precautionary_stop[]
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // end::precautionary_stop[]

    // Login to the device for access rights to certain methods
    visionaryControl->login(IAuthentication::UserLevel::SERVICE, "Visionary-T-Mini@");
    std::shared_ptr<visionary::NetLink> udpSocket;

    // tag::tcp_settings[]
    if (transportProtocol == "TCP")
    {
        // configure the data stream
        // the methods immediately write the setting to the device
        // set protocol and device port
        setTransportProtocol(visionaryControl, transportProtocol); // TCP
        setBlobTcpPort(visionaryControl, streamingPort);
    }
    // end::tcp_settings[]

   
    // login / logout always need to form blocks because login chnages to config mode (no streaming active) and logout
    // returns to RUN mode (streaming)
    visionaryControl->logout();

    // start the image acquisition and continuously receive frames
    // tag::start_acquisition[]
    if (!visionaryControl->startAcquisition())
    {
        std::fprintf(stderr, "Failed to start acquisition.\n");

        return ExitCode::eControlCommunicationError;
    }
    // end::start_acquisition[]

    std::shared_ptr<VisionaryData>    pDataHandler = nullptr;
    std::unique_ptr<FrameGrabberBase> pFrameGrabber = nullptr;

    if (transportProtocol == "TCP")
    {
        std::shared_ptr<VisionaryData> pDataHandler     = visionaryControl->createDataHandler();
        std::unique_ptr<FrameGrabberBase> pFrameGrabber = visionaryControl->createFrameGrabber();

        while (true)
        {
            if (!pFrameGrabber->genGetNextFrame(pDataHandler))
            {
                std::cerr << "[Error] Frame timeout\n";
                break;
            }

            // Genera point cloud
            std::vector<PointXYZ> pointCloud;
            pDataHandler->generatePointCloud(pointCloud);

            int width  = pDataHandler->getWidth();
            int height = pDataHandler->getHeight();

            cv::Mat img = processPointCloud(pointCloud, width, height, originPlaneProjecton, normal, scale, originCutPlanes, inclinationCutPlanes);

            cv::imshow("PointCloud", img);

            if (cv::waitKey(30) == 'q')
                break;
        }
        cv::destroyAllWindows();
    }

    //-----------------------------------------------

    for (unsigned i = 0u; i < numberOfFrames; ++i)
    {
        // tag::tcp_acquisition[]
        if (transportProtocol == "TCP")
        {
            if (!pFrameGrabber->genGetNextFrame(pDataHandler))
            {
                visionaryControl->stopAcquisition();

                std::fprintf(stderr, "Frame timeout in continuous mode after %u frames\n", i);

                return ExitCode::eFrameTimeout;
            }
            else
            {
                std::printf("Frame received in continuous mode, frame #%" PRIu32 "\n", pDataHandler->getFrameNum());
                if (storeData)
                {
                    // write the frame to disk
                    //writeFrame(visionaryType, *pDataHandler, filePrefix);

                    // Convert data to a point cloud
                    std::vector<PointXYZ> pointCloud;
                    pDataHandler->generatePointCloud(pointCloud);
                    pDataHandler->transformPointCloud(pointCloud);

                    // Write point cloud to PLY
                    const std::string framePrefix = std::to_string(pDataHandler->getFrameNum());
                    std::string       plyFilePath = framePrefix + "-pointcloud.ply";
                    const char* cPlyFilePath = plyFilePath.c_str();
                    std::printf("Writing frame to %s\n", cPlyFilePath);

                    if (visionaryType == VisionaryType::eVisionaryS)
                        PointCloudPlyWriter::WriteFormatPLY(cPlyFilePath, pointCloud, pDataHandler->getRGBAMap(), true);
                    else
                        PointCloudPlyWriter::WriteFormatPLY(cPlyFilePath, pointCloud, pDataHandler->getIntensityMap(), true);
                    std::printf("Finished writing frame to %s\n", cPlyFilePath);
                }
            }
        }
        // end::tcp_acquisition[]
    }

    //-----------------------------------------------
    // tag::stop_acquisition[]
    if (!visionaryControl->stopAcquisition())
    {
        std::fprintf(stderr, "Failed to stop acquisition.\n");

        return ExitCode::eControlCommunicationError;
    }
    // end::stop_acquisition[]

    if (transportProtocol == "TCP")
    {
        // delete the frame grabber
        // this operation is blocking, since the frame grabber thread is joined which takes up to 5s
        // (actually since the acquisition is still running, the thread will wake up and join only one frame time of ~33ms)
        // tag::release_frame_grabber[]
        pFrameGrabber.reset();
        // end::release_frame_grabber[]
    }
    // tag::close_control_channel[]
    visionaryControl->close();
    // end::close_control_channel[]
    std::cout << "Logout and close.\n";

    return ExitCode::eOk;
}

bool loadDataFromJSON(
    std::string& transportProtocol,
    std::string& deviceIpAddr,
    std::string& receiverIp,
    bool& storeData,
    std::string& filePrefix,
    std::uint16_t& streamingPort,
    unsigned& cnt,
    visionary::VisionaryType& visionaryType,
    double& scale,
    Eigen::Vector3d& originPlaneProjection,
    Eigen::Vector3d& normal,
    std::vector<Vector3d> originCutPlanes,
    std::vector<Vector3d> inclinationCutPlanes)
{
    std::cout << "[Step] Inizio lettura file JSON" << std::endl;

    Config cfg;
    try 
    {
        cfg = readJSON("config.json");  // assicurati che readJSON possa lanciare eccezioni
    }
    catch (const std::exception& e) 
    {
        std::cerr << "[Error] Impossibile leggere config.json: " << e.what() << std::endl;
        return false;
    }

    try 
    {
        // Parametri SICK
        transportProtocol = cfg.sick_settings.transportProtocol;
        deviceIpAddr      = cfg.sick_settings.deviceIpAddr;
        receiverIp        = cfg.sick_settings.receiverIp;
        storeData         = cfg.sick_settings.storeData;
        filePrefix        = cfg.sick_settings.filePrefix;
        streamingPort     = cfg.sick_settings.streamingPort;
        cnt               = cfg.sick_settings.cnt;

        // Parametri processing Point Cloud
        scale                 = cfg.cloud_settings.scale;
        originPlaneProjection = cfg.cloud_settings.originPlaneProjection;
        normal                = cfg.cloud_settings.normal;
        originCutPlanes       = cfg.cloud_settings.originCutPlanes;
        inclinationCutPlanes  = cfg.cloud_settings.inclinationCutPlanes;
    }
    catch (const std::exception& e) 
    {
        std::cerr << "[Error] Errore nella lettura dei parametri: " << e.what() << std::endl;
        return false;
    }

    std::cout << "[Step] Lettura file JSON completata\n" << std::endl;
    return true;
}






// Funzione helper per caricare la PointCloud
PointCloud loadPointCloud(const std::string& filename, char delim, const char* outfile_name = nullptr, FILE** outfile_ptr = nullptr)
{
    // Apertura file di output se specificato
    FILE* outfile = stdout;
    if (outfile_name) {
        int err = fopen_s(&outfile, outfile_name, "w");
        if (err) {
            throw std::runtime_error("Cannot open outfile '" + std::string(outfile_name) + "'!");
        }
    }
    if (outfile_ptr) {
        *outfile_ptr = outfile;
    }

    // Legge la PointCloud dal file
    PointCloud X;
    int errorcode = X.readFromFile(filename.c_str(), delim);
    if (errorcode == 2) {
        throw std::runtime_error("Wrong file format of infile '" + filename + "'!");
    }
    else if (errorcode != 0) {
        throw std::runtime_error("Cannot read infile '" + filename + "'!");
    }
    if (X.points.size() < 2) {
        throw std::runtime_error("Point cloud has less than two points");
    }
    return X;
}

// Funzione per testare il processing con una PointCloud caricata da file 
int staticTest()
{
    //Parametri
	PointCloud X;
    std::vector<PointXYZ> cloud;

    char opt_delim = ',';
    char* outfile_name = NULL;

    // Carica la PointCloud da file o da generatore
    std::string filename = "Gabbiatrice.dat";


    FILE* outfile = nullptr;
    try
    {
        X = loadPointCloud(filename, opt_delim, outfile_name, &outfile);
        // ora X contiene la point cloud caricata
        // outfile contiene il file di output (stdout se non specificato)
    }
    catch (const std::exception& e)
    {
        std::cerr << "Errore: " << e.what() << std::endl;
        return 1;
    }

    for(const auto& pt : X.points)
    {
        PointXYZ p;
        p.x = static_cast<float>(pt.x);
        p.y = static_cast<float>(pt.y);
        p.z = static_cast<float>(pt.z);
        cloud.push_back(p);
	}

    // Tagli per posizione (origine piani di taglio)
    std::vector<Vector3d> originCutPlanes = {
        Vector3d(187.899, 206.022, 789.286),
        Vector3d(347.869, -13.978, 789.286)
    };

    // Tagli per inclinazione (normali dei piani)
    std::vector<Vector3d> inclinationCutPlanes = {
        Vector3d(360, 180.05, 270),
        Vector3d(10, 270, 230)
    };

	//Eigen::Vector3d origin(0.0, 400.0, 580.0);
	Eigen::Vector3d origin(0.0, 0.0, 0.0);
	Eigen::Vector3d normal(0.0, 0.0, 1.0);

    // Processa e visualizza l'immagine
    cv::Mat img = testProcessPointCloud(
        cloud,
        640,
        640,
        origin,
        normal,
        1.0
    );

    //cv::imshow("finalAnnotatedImage", img);

    //cv::waitKey(0);

    //cv::destroyAllWindows();
}



int
main() 
{
    //
	//  Parametri configurazione camera SICK 
    // 
    
    // Protocollo di trasporto usato per la comunicazione con il dispositivo
    std::string transportProtocol;
    // Indirizzo IP del sensore
    std::string deviceIpAddr;
    // Indirizzo IP del PC ricevente, rilevante solo per connessioni UDP
    std::string receiverIp;
    // Flag che indica se salvare i dati acquisiti su file
    bool storeData;
    // Prefisso dei file di output se si salvano i dati
    std::string filePrefix;
    // Porta della connessione per il flusso dati dal dispositivo
    std::uint16_t streamingPort;
    // Numero di frame da acquisire prima di fermarsi
    unsigned cnt;
    // sensore da usare -> Visionary T-Mini
    visionary::VisionaryType visionaryType(visionary::VisionaryType::eVisionaryTMini);

    //
    // Parametri per il processing delle Point cloud
    //

	// Scala per la proiezione sul piano
    double scale;
    // Piano di proiezione
    Eigen::Vector3d originPlaneProjection, normal;
    // Piani di taglio
    std::vector<Vector3d> originCutPlanes, inclinationCutPlanes;

    // Stampa versioni delle librerie
    std::cout << "=== Library Versions ===" << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "Open3D version: " << OPEN3D_VERSION << std::endl;
    std::cout << "=========================\n" << std::endl;

    // Legge i dati dal JSON
    /*if (!loadDataFromJSON
    (
        transportProtocol, deviceIpAddr, receiverIp, storeData, filePrefix, streamingPort, cnt, visionaryType,
        scale, originPlaneProjection, normal, originCutPlanes, inclinationCutPlanes
    ))
    {
        return static_cast<int>(ExitCode::eParamError);
    }*/

    ExitCode exitCode = ExitCode::eOk;

    staticTest();

    // Inizia la trasmissione dei dati
    //exitCode = runContinuousStreamingDemo
    //(
    //    visionaryType, transportProtocol, deviceIpAddr, receiverIp,streamingPort, cnt, filePrefix, storeData,
    //    scale, originPlaneProjection, normal, originCutPlanes, inclinationCutPlanes
    //);

    std::cout << "exit code " << static_cast<int>(exitCode) << '\n';

    return static_cast<int>(exitCode);
}
