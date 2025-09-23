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
#include <include/FrameGrabber.h>
#include <include/NetLink.h>
#include <include/PointCloudPlyWriter.h>
#include <include/PointXYZ.h>
#include <include/VisionaryControl.h>
#include <include/VisionaryType.h>
#include <pcl/pcl_config.h>  


// =======================================================
// Header locali del progetto
// =======================================================
#include "include/BlobServerConfig.h"
#include "include/UdpParsing.h"
#include "include/exitcodes.h"
#include "include/Config.h"
#include "include/PointCloudProcessor.h"
#include "include/Matching3D.h" 

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

// Funzione per acquisire un singolo frame e ottenere la point cloud
std::vector<PointXYZ> acquireSingleFramePointCloud(
    visionary::VisionaryType visionaryType,
    const std::string&       ipAddress,
    std::uint16_t            streamingPort)
{
    using namespace visionary;

    std::vector<PointXYZ> pointCloud;

    // Crea il controllo per il dispositivo
    std::shared_ptr<VisionaryControl> visionaryControl = std::make_shared<VisionaryControl>(visionaryType);

    // Connessione al dispositivo
    std::cout << "[Step] Connessione al dispositivo " << ipAddress << "...\n";
    if (!visionaryControl->open(ipAddress))
    {
        std::fprintf(stderr, "[Errore] Impossibile connettersi al dispositivo.\n");
        return pointCloud; // Ritorna vector vuoto in caso di errore
    }
    std::cout << "[Step] Connessione stabilita.\n";

    // Stop acquisizione (precauzionale)
    if (!visionaryControl->stopAcquisition())
    {
        std::fprintf(stderr, "[Errore] Impossibile fermare l'acquisizione.\n");
        visionaryControl->close();
        return pointCloud;
    }

    // Attendi un momento per assicurarsi che lo stop sia propagato
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Login per i diritti di accesso
    visionaryControl->login(IAuthentication::UserLevel::SERVICE, "Visionary-T-Mini@");

    // Configura TCP
    setTransportProtocol(visionaryControl, "TCP");
    setBlobTcpPort(visionaryControl, streamingPort);

    // Logout per tornare in modalità RUN
    visionaryControl->logout();

    // Avvia acquisizione
    if (!visionaryControl->startAcquisition())
    {
        std::fprintf(stderr, "[Errore] Impossibile avviare l'acquisizione.\n");
        visionaryControl->close();
        return pointCloud;
    }

    // Crea data handler e frame grabber
    std::shared_ptr<VisionaryData> pDataHandler = visionaryControl->createDataHandler();
    std::unique_ptr<FrameGrabberBase> pFrameGrabber = visionaryControl->createFrameGrabber();

    // Acquisisce un singolo frame
    if (!pFrameGrabber->genGetNextFrame(pDataHandler))
    {
        std::fprintf(stderr, "[Errore] Timeout nel ricevere il frame.\n");
    }
    else
    {
        std::printf("Frame ricevuto, frame #%" PRIu32 "\n", pDataHandler->getFrameNum());

        // Genera point cloud dal frame
        pDataHandler->generatePointCloud(pointCloud);
        pDataHandler->transformPointCloud(pointCloud);

        std::printf("Point cloud generata con %zu punti.\n", pointCloud.size());
    }

    // Cleanup
    visionaryControl->stopAcquisition();
    pFrameGrabber.reset();
    visionaryControl->close();

    return pointCloud;
}

bool 
loadDataFromJSON(
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
        cfg = readJSON("resources/config.json");  // assicurati che readJSON possa lanciare eccezioni
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
PointCloud
loadPointCloud(
    const std::string& filename,
    char delim, 
    const char* outfile_name = nullptr,
    FILE** outfile_ptr = nullptr)
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


// Funzione principale per il sistema di allineamento
void startSystemAlign(
    visionary::VisionaryType visionaryType, 
    std::string&             deviceIpAddr, 
    std::uint16_t            streamingPort)
{
    // Acquisisco un frame dalla camera
    std::vector<PointXYZ> pointCloud = acquireSingleFramePointCloud(visionaryType, deviceIpAddr, streamingPort);

    // Verifica che la point cloud sia stata acquisita correttamente
    if (pointCloud.empty())
    {
        std::fprintf(stderr, "[Errore] Impossibile ottenere la point cloud.\n");
        return;
    }
    else
    {
        std::cout << "[Step] Point cloud acquisita con " << pointCloud.size() << " punti.\n";
    }

    // Converte la point cloud ottenuta per iniziare il matching 3D
    pcl::PointCloud<PointType>::Ptr scenePointCloud(new pcl::PointCloud<PointType>());
    scenePointCloud->points.resize(pointCloud.size());

    // Usa std::transform per copiare e convertire tutti i punti in una volta
    std::transform(pointCloud.begin(), pointCloud.end(),
        scenePointCloud->points.begin(),
        [](const PointXYZ& point) 
        {
            PointType pclPoint;
            pclPoint.x = point.x;
            pclPoint.y = point.y;
            pclPoint.z = point.z;
            return pclPoint;
        });

    // Imposta le proprietà della point cloud
    scenePointCloud->width = scenePointCloud->points.size();
    scenePointCloud->height = 1;
    scenePointCloud->is_dense = true;

    std::string model_filename_ = "data/BordoLegatriceRandom.pcd";
    Eigen::Matrix3f transformation_matrix;
    Eigen::Vector3f translation_vector;

    startMatching3DFromCamera(transformation_matrix, translation_vector, model_filename_, scenePointCloud);

    // Ottieni le matrici 
    printf("\n");
    printf("            | %6.3f %6.3f %6.3f | \n", transformation_matrix(0, 0), transformation_matrix(0, 1), transformation_matrix(0, 2));
    printf("        R = | %6.3f %6.3f %6.3f | \n", transformation_matrix(1, 0), transformation_matrix(1, 1), transformation_matrix(1, 2));
    printf("            | %6.3f %6.3f %6.3f | \n", transformation_matrix(2, 0), transformation_matrix(2, 1), transformation_matrix(2, 2));
    printf("\n");
    printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation_vector(0), translation_vector(1), translation_vector(2));
}


// Funzione che fa partire la modilita per ottenere i piani di taglio 
int cutTesting()
{
    //Parametri
    PointCloud X;
    std::vector<PointXYZ> cloud;

    char opt_delim = ',';
    char* outfile_name = NULL;

    // Carica la PointCloud da file o da generatore
    std::string filename = "data/Gabbiatrice.dat";


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

    for (const auto& pt : X.points)
    {
        PointXYZ p;
        p.x = static_cast<float>(pt.x);
        p.y = static_cast<float>(pt.y);
        p.z = static_cast<float>(pt.z);
        cloud.push_back(p);
    }

    //Eigen::Vector3d origin(0.0, 400.0, 580.0);
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    Eigen::Vector3d normal(0.0, 0.0, 1.0);

    // Processa e visualizza l'immagine
    cv::Mat img = start3dPointCloudCut(
        cloud,
        640,
        640,
        origin,
        normal,
        1.0
    );
}

// Elabora la point cloud e restituisce prima in 3D poi in due 2D i punti da legare
int processPointCloudFromFile()
{
    //Parametri
	PointCloud X;
    std::vector<PointXYZ> cloud;

    char opt_delim = ',';
    char* outfile_name = NULL;

    // Carica la PointCloud da file o da generatore
    std::string filename = "data/Gabbiatrice.dat";


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

    // Origine dei piani di taglio
    std::vector<Vector3d> originCutPlanes = {
        Vector3d(197.899, -13.978,  789.286),   
        Vector3d(187.899, 206.022,  789.286)
    };

    // Inclinazioni dei piani di taglio
    std::vector<Vector3d> inclinationCutPlanes = {
        Vector3d(40, 370, 0),
        Vector3d(0,0,-89.97)
    };

    // Origine e normale del piano per la proiezione su Immgaine 2D
	Eigen::Vector3d origin(0.0, 0.0, 0.0);
	Eigen::Vector3d normal(0.0, 0.0, 1.0);

    cv::Mat img = processPointCloud(
        cloud,
        640,
        640,
        origin,
        normal,
        1.0,
        originCutPlanes,
        inclinationCutPlanes
	);

    cv::imshow("finalAnnotatedImage", img);

    cv::waitKey(0);

    cv::destroyAllWindows();
}

// Tramite i 3 piani di taglio taglia la point cloud e mostra con una visualizzazione 3D la testa della legatrice
int getBinderPointCloud()
{
    //Parametri
    PointCloud pointCloud;

    char opt_delim = ',';
    char* outfile_name = NULL;

    // Carica la PointCloud da file o da generatore
    std::string filename = "data/Gabbiatrice.dat";
    FILE* outfile = nullptr;
    try
    {
        pointCloud = loadPointCloud(filename, opt_delim, outfile_name, &outfile);
        // ora pointCloud contiene la point cloud caricata
        // outfile contiene il file di output (stdout se non specificato)
    }
    catch (const std::exception& e)
    {
        std::cerr << "Errore: " << e.what() << std::endl;
        return 1;
    }

    // Tagli per ottenere la sagoma della Legatrice

    std::vector<Vector3d> originCutPlanes = {
        Vector3d(187.899, 206.022, 776.046),   //Taglio punti sotto la legatrice
        Vector3d(187.899, 206.022, 2506.05),   //Taglio punti sopra la legatrice
        Vector3d(187.899, 86.022, 824.946)     //Taglio punti laterali a destra
    };

    std::vector<Vector3d> inclinationCutPlanes = {
        Vector3d(-30,-500,-70),
        Vector3d(-80,100,-50),
        Vector3d(-30,500,300),
    };

    show3dBinderPointCloud(pointCloud, originCutPlanes, inclinationCutPlanes);

    return 0;
}

// Funzione che effettua l image matching per cercare la legatrice e stampa a schermo la matrice dell inclinazione e il vettore della traslazione
void TestFindBinderHead()
{
    Eigen::Matrix3f transformation_matrix;
    Eigen::Vector3f translation_vector;

    startMatching3DFromFile(transformation_matrix, translation_vector);

    printf("\n");
    printf("            | %6.3f %6.3f %6.3f | \n", transformation_matrix(0, 0), transformation_matrix(0, 1), transformation_matrix(0, 2));
    printf("        R = | %6.3f %6.3f %6.3f | \n", transformation_matrix(1, 0), transformation_matrix(1, 1), transformation_matrix(1, 2));
    printf("            | %6.3f %6.3f %6.3f | \n", transformation_matrix(2, 0), transformation_matrix(2, 1), transformation_matrix(2, 2));
    printf("\n");
    printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation_vector(0), translation_vector(1), translation_vector(2));

}

int main() 
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
    std::cout << "PCL    version: " << PCL_VERSION_PRETTY << std::endl;
    std::cout << "=========================\n" << std::endl;

    // Legge i dati dal JSON
    if (!loadDataFromJSON(
        transportProtocol, deviceIpAddr, receiverIp, storeData, filePrefix, streamingPort, cnt, visionaryType,
        scale, originPlaneProjection, normal, originCutPlanes, inclinationCutPlanes
    ))
    {
        return static_cast<int>(ExitCode::eParamError);
    }

    ExitCode exitCode = ExitCode::eOk;


    //
    //  Funzioni
    //

    startSystemAlign(visionaryType, deviceIpAddr, streamingPort);

    //TestFindBinderHead();
    
    //cutTesting();

    //processPointCloudFromFile();

    //getBinderPointCloud();

    // Inizia la trasmissione dei dati
    exitCode = runContinuousStreamingDemo(
        visionaryType, transportProtocol, deviceIpAddr, receiverIp,streamingPort, cnt, filePrefix, storeData,
        scale, originPlaneProjection, normal, originCutPlanes, inclinationCutPlanes
    );

    std::cout << "exit code " << static_cast<int>(exitCode) << '\n';

    return static_cast<int>(exitCode);
}
