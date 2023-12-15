#include <cmath>
#include <vector>
#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "BodyBasics.h"

using namespace std;

const char OPEN_GRIPPER = 'O';
const char CLOSE_GRIPPER = 'C';
const char NULL_GRIPPER = 'N';
const char MOVE_UP = 'U';
const char MOVE_DOWN = 'D';
const char MOVE_LEFT = 'L';
const char MOVE_RIGHT = 'R';

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;
bool tracked[6] = { false, false, false , false , false , false };
//proyectoRamssangh
static const float c_joinThickness = 4.5f;

#define NOISE 0.1f

const float euclidianDistance(const CameraSpacePoint& one, const CameraSpacePoint& two)
{
    return sqrt(pow(two.X - one.X, 2) + pow(two.Y - one.Y, 2) + pow(two.Z - one.Z, 2));
}


const float calibracionDistance(const CameraSpacePoint& one, const CameraSpacePoint& two)
{
    return sqrt(pow(two.X - one.X, 2) + pow(two.Y - one.Y, 2) + pow(two.Z - one.Z, 2));
}




/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(
    _In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    AllocConsole();

    CBodyBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CBodyBasics::CBodyBasics() : m_hWnd(NULL),
m_nStartTime(0),
m_nLastCounter(0),
m_nFramesSinceUpdate(0),
m_fFreq(0),
m_nNextStatusTime(0LL),
m_currentJoint(0LL),
m_pKinectSensor(NULL),
m_pCoordinateMapper(NULL),
m_pBodyFrameReader(NULL),
m_pD2DFactory(NULL),
m_pRenderTarget(NULL),
m_pBrushJointTracked(NULL),
m_pBrushJointInferred(NULL),
m_pBrushBoneTracked(NULL),
m_pBrushBoneInferred(NULL),
m_pBrushHandClosed(NULL),
m_pBrushHandOpen(NULL),
m_pBrushHandLasso(NULL),
m_prevJoints(NULL),
m_setPrevJoints(NULL),
m_nPrevJoints(30LL),
m_populatedJoints(false)

{
    m_prevJoints = new CameraSpacePoint[m_nPrevJoints];
    m_setPrevJoints = new bool[m_nPrevJoints];

    ClearPrevJoints();

    LARGE_INTEGER qpf = { 0 };
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

    console = GetStdHandle(STD_OUTPUT_HANDLE);

    printConsole("-- RAMSSAHG DEBUGGER --\n");
    printConsole(std::to_string(123) + std::to_string(456));

    connectArduino(L"COM1");
}

/// <summary>
/// Destructor
/// </summary>
CBodyBasics::~CBodyBasics()
{
    DiscardDirect2DResources();

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);

    FreeConsole();
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>


int CBodyBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG msg = { 0 };
    WNDCLASS wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra = DLGWINDOWEXTRA;
    wc.hCursor = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc = DefDlgProcW;
    wc.lpszClassName = L"BodyBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CBodyBasics::MessageRouter,
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CBodyBasics::Update()
{
    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = { 0 };

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            ProcessBody(nTime, BODY_COUNT, ppBodies);
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CBodyBasics* pThis = NULL;

    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CBodyBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CBodyBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CBodyBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
    case WM_INITDIALOG:
    {
        // Bind application window handle
        m_hWnd = hWnd;

        // Init Direct2D
        D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

        // Get and initialize the default Kinect sensor
        InitializeDefaultSensor();
    }
    break;

    // If the titlebar X is clicked, destroy app
    case WM_CLOSE:
        DestroyWindow(hWnd);
        break;

    case WM_DESTROY:
        // Quit the main message pump
        PostQuitMessage(0);
        break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// </summary>
void CBodyBasics::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
    vector<D2D1_POINT_2F> ballPositions;
    const int numEllipses = 10;
    // Radio del círculo
    float radius = 250.0f;
    // Centro del círculo
    //const D2D1_POINT_2F center = D2D1::Point2F(300, 300);
    if (m_hWnd)
    {
        HRESULT hr = EnsureDirect2DResources();

        if (SUCCEEDED(hr) && m_pRenderTarget && m_pCoordinateMapper)
        {
            m_pRenderTarget->BeginDraw();
            m_pRenderTarget->Clear();

            RECT rct;
            GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
            int width = rct.right;
            int height = rct.bottom;

            for (int i = 0; i < nBodyCount; ++i)
            {
                IBody* pBody = ppBodies[i];
                if (pBody)
                {
                    BOOLEAN bTracked = false;
                    hr = pBody->get_IsTracked(&bTracked);

                    if (SUCCEEDED(hr) && bTracked)
                    {
                        Joint joints[JointType_Count];
                        D2D1_POINT_2F jointPoints[JointType_Count];
                        HandState leftHandState = HandState_Unknown;

                        pBody->get_HandLeftState(&leftHandState);

                        hr = pBody->GetJoints(_countof(joints), joints);
                        if (SUCCEEDED(hr))
                        {
                            if (calibrando)
                            {
                                //lo lleva a la pantalla
                                for (int j = 0; j < _countof(joints); ++j)
                                {
                                    jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
                                }

                                DrawBody(joints, jointPoints);
                                DrawHand(leftHandState, jointPoints[JointType_HandLeft]);

                                D2D1_POINT_2F shoulderPoint = jointPoints[JointType_ShoulderLeft];
                                D2D1_POINT_2F handPoints = jointPoints[JointType_HandLeft];
                                D2D1_POINT_2F elbowPoints = jointPoints[JointType_ElbowLeft];
                                // Need to get the distance between the shoulder and the hand joints
                                
                                float dx = shoulderPoint.x - elbowPoints.x;
                                float dy = shoulderPoint.y - elbowPoints.y;
                                radius = sqrt(dx * dx + dy * dy);

                                dx = elbowPoints.x - handPoints.x;
                                dy = elbowPoints.y - handPoints.y;

                                radius += sqrt(dx * dx + dy * dy);

                                std::cout << radius << std::endl;

                                D2D1_POINT_2F center = shoulderPoint;
                                float angle1 = 0;

                                

                                bool arrayCol[6];
                                vector<pair<float, float>> pointPos;
                                vector<pair<float, float>> ellipsePos;

                                for (int i = 0; i < (numEllipses / 2) + 1; ++i) {

                                    float angle = static_cast<float>(i) * (6.28318530718f / numEllipses); // Calcula el ángulo entre las elipses
                                    float angle1 = +45;
                                    float x = center.x + radius * cosf(angle); // Calcula la coordenada x basada en el ángulo y el radio
                                    float y = center.y + radius * sinf(angle); // Calcula la coordenada y basada en el ángulo y el radio
                                    


                                    D2D1_POINT_2F point = D2D1::Point2F(x, y);
                                    pointPos.push_back(make_pair(point.x, point.y));
                                    D2D1_ELLIPSE ellipse = D2D1::Ellipse(point, c_joinThickness * 5, c_joinThickness * 5);
                                    ellipsePos.push_back(make_pair(ellipse.point.x, ellipse.point.y));
                                    
                                    //printConsole(std::to_string(tracked[i]));

                                    if(!tracked[i])
                                         m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
                                    
                                    ballPositions.push_back(point); // Guarda la posición de la bola en el vector
                                }
                                // Llamar a la función de detección de colisión para las articulaciones relevantes
                                for (int i = 0; i < _countof(joints); ++i) {
                                    if (!tracked[i])
                                        jointPoints[i] = BodyToScreen(joints[i].Position, width, height);
                                }

                                CheckCollisionAndChangeColor(jointPoints[JointType_HandLeft], ballPositions, tracked);

                                // Imprime las posiciones de las bolas
                                for (const auto& position : ballPositions) {
                                    std::cout << "X: " << position.x << ", Y: " << position.y << std::endl;
                                }
                            }
                            else {

                            }
                            if (!m_populatedJoints)
                            {
                                m_prevJoints[m_currentJoint] = joints[JointType_ElbowLeft].Position;
                                m_setPrevJoints[m_currentJoint++] = true;
                                //ejecuta 30 veces
                                if (PopulatedPrevJoints())
                                    m_populatedJoints = true;
                                //en la 31 lo vuelve falso, el siguiente if se ejecuta una vez
                            }

                            for (int j = 0; j < _countof(joints); ++j)
                            {
                                jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
                            }

                            DrawBody(joints, jointPoints);
                            DrawHand(leftHandState, jointPoints[JointType_HandLeft]);

                            if (m_populatedJoints && JointIsMoving(joints[JointType_ElbowLeft].Position))
                            {
                                CameraSpacePoint directions;
                                //DIRECTIONS XYZ
                                directions.X = joints[JointType_ElbowLeft].Position.X - m_prevJoints[0].X; // Delta X
                                directions.Y = joints[JointType_ElbowLeft].Position.Y - m_prevJoints[0].Y; // Delta Y
                                directions.Z = joints[JointType_ElbowLeft].Position.Z - m_prevJoints[0].Z; // Delta Z - not considered
                                
                                printConsole(std::to_string(directions.X) + "\n");
                                printConsole(std::to_string(directions.Y) + "\n");
                                printConsole(std::to_string(directions.Z) + "\n");

                                if (directions.X > 0.0f)
                                {
                                    printConsole(std::to_string(directions.X) + "\n");

                                    if (directions.X > NOISE)
                                        sendToArduino(MOVE_RIGHT);

                                }

                                if (directions.X > -0.70 && directions.Y > 0.70)
                                {
                                    printConsole("hola");
                                }
                                else
                                {
                                    printConsole(std::to_string(directions.X) + "\n");
                                    if (abs(directions.X) > NOISE)
                                        sendToArduino(MOVE_LEFT);
                                }
                                if (directions.Y > 0.0f)
                                {
                                    printConsole(std::to_string(directions.Y) + "\n");

                                    if (directions.Y > NOISE)
                                        sendToArduino(MOVE_UP);
                                }
                                else
                                {
                                    printConsole(std::to_string(directions.Y) + "\n");
                                    if (abs(directions.Y) > NOISE)
                                        sendToArduino(MOVE_DOWN);
                                }

                                //se vuelve todo 0
                                ClearPrevJoints(); // Setting Previous Joints to zero for security


                            }
                        }
                    }
                }
            }

            hr = m_pRenderTarget->EndDraw();

            // Device lost, need to recreate the render target
            // We'll dispose it now and retry drawing
            if (D2DERR_RECREATE_TARGET == hr)
            {
                hr = S_OK;
                DiscardDirect2DResources();
            }
        }

        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = { 0 };
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }
}



void CBodyBasics::CheckCollisionAndChangeColor(const D2D1_POINT_2F& jointPoint, vector<D2D1_POINT_2F>& ballPositions, bool *tracked)
{
    const float collisionRadius = 25.0f; // Radio de detección de colisión
    for (int i = 0; i < ballPositions.size(); i++) {
        float dx = jointPoint.x - ballPositions[i].x;
        float dy = jointPoint.y - ballPositions[i].y;
        float distance = sqrt(dx * dx + dy * dy);
        if (distance < collisionRadius) {
            tracked[i] = true;
            // Colisión detectada, cambiar el color de la bola
            D2D1_ELLIPSE ellipse = D2D1::Ellipse(ballPositions[i], c_joinThickness * 5, c_joinThickness * 5);
            //m_pRenderTarget->FillEllipse(ellipse, m_pBrushBoneInferred); // Usar un pincel de color diferente
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushBallCollision); // Usar pincel rojo
            break; // Comentar esta línea si se desea detectar colisiones con múltiples bolas simultáneamente
        }
    }
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CBodyBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Ensure necessary Direct2d resources are created
/// </summary>
/// <returns>S_OK if successful, otherwise an error code</returns>
HRESULT CBodyBasics::EnsureDirect2DResources()
{
    HRESULT hr = S_OK;

    if (m_pD2DFactory && !m_pRenderTarget)
    {
        RECT rc;
        GetWindowRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rc);

        int width = rc.right - rc.left;
        int height = rc.bottom - rc.top;
        D2D1_SIZE_U size = D2D1::SizeU(width, height);
        D2D1_RENDER_TARGET_PROPERTIES rtProps = D2D1::RenderTargetProperties();
        rtProps.pixelFormat = D2D1::PixelFormat(DXGI_FORMAT_B8G8R8A8_UNORM, D2D1_ALPHA_MODE_IGNORE);
        rtProps.usage = D2D1_RENDER_TARGET_USAGE_GDI_COMPATIBLE;

        // Create a Hwnd render target, in order to render to the window set in initialize
        hr = m_pD2DFactory->CreateHwndRenderTarget(
            rtProps,
            D2D1::HwndRenderTargetProperties(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), size),
            &m_pRenderTarget);

        if (FAILED(hr))
        {
            SetStatusMessage(L"Couldn't create Direct2D render target!", 10000, true);
            return hr;
        }

        // light green
        //m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0.27f, 0.75f, 0.27f), &m_pBrushJointTracked);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Yellow, 1.0f), &m_pBrushJointInferred);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 1.0f), &m_pBrushBoneTracked);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Gray, 1.0f), &m_pBrushBoneInferred);

        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red, 0.5f), &m_pBrushHandClosed);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Green, 0.5f), &m_pBrushHandOpen);
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Blue, 0.5f), &m_pBrushHandLasso);

        // Colorcito madafaka 
        m_pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::Red), &m_pBrushBallCollision);

    }

    return hr;
}

/// <summary>
/// Dispose Direct2d resources
/// </summary>
void CBodyBasics::DiscardDirect2DResources()
{
    SafeRelease(m_pRenderTarget);

    SafeRelease(m_pBrushJointTracked);
    SafeRelease(m_pBrushJointInferred);
    SafeRelease(m_pBrushBoneTracked);
    SafeRelease(m_pBrushBoneInferred);

    SafeRelease(m_pBrushHandClosed);
    SafeRelease(m_pBrushHandOpen);
    SafeRelease(m_pBrushHandLasso);
}

/// <summary>
/// Converts a body point to screen space
/// </summary>
/// <param name="bodyPoint">body point to tranform</param>
/// <param name="width">width (in pixels) of output buffer</param>
/// <param name="height">height (in pixels) of output buffer</param>
/// <returns>point in screen-space</returns>
D2D1_POINT_2F CBodyBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
    // Calculate the body's position on the screen
    DepthSpacePoint depthPoint = { 0 };
    m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

    float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
    float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

    return D2D1::Point2F(screenPointX, screenPointY);
}

/// <summary>
/// Draws a body
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
void CBodyBasics::DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints)
{
    // Draw the bones

    /* Only drawing left arm */

    /*
        JointType_ShoulderLeft	= 4,
        JointType_ElbowLeft	= 5,
        JointType_WristLeft	= 6,
        JointType_HandLeft	= 7,
        JointType_ShoulderRight	= 8,
        JointType_ElbowRight	= 9,
        JointType_WristRight	= 10,
        JointType_HandRight	= 11,
        JointType_SpineShoulder	= 20,
     */

    unsigned int preferredJoints[] = { JointType_ShoulderLeft, JointType_ElbowLeft, JointType_WristLeft, JointType_HandLeft };
        /*JointType_ShoulderRight, JointType_ElbowRight, JointType_WristRight, JointType_HandRight};
        */
    DrawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft);
    DrawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft);
    DrawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft);
    DrawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft);
    /*DrawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight);
    DrawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight);
    DrawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight);
    DrawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight);
    */


    // Draw the Joints
    for (int i = 0; i < sizeof(preferredJoints) / sizeof(preferredJoints[0]); ++i)
    {
        D2D1_ELLIPSE ellipse = D2D1::Ellipse(pJointPoints[preferredJoints[i]], c_JointThickness * 4, c_JointThickness * 4);

        if (pJoints[preferredJoints[i]].TrackingState == TrackingState_Inferred)
        {
            // m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointInferred);
        }
        else if (pJoints[preferredJoints[i]].TrackingState == TrackingState_Tracked)
        {
            m_pRenderTarget->FillEllipse(ellipse, m_pBrushJointTracked);
        }
    }
}

/// <summary>
/// Draws one bone of a body (joint to joint)
/// </summary>
/// <param name="pJoints">joint data</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="pJointPoints">joint positions converted to screen space</param>
/// <param name="joint0">one joint of the bone to draw</param>
/// <param name="joint1">other joint of the bone to draw</param>
void CBodyBasics::DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1)
{
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneTracked, c_TrackedBoneThickness * DRAWING_SCALE);
    }
    else
    {
        m_pRenderTarget->DrawLine(pJointPoints[joint0], pJointPoints[joint1], m_pBrushBoneInferred, c_InferredBoneThickness * DRAWING_SCALE);
    }
}

/// <summary>
/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
/// </summary>
/// <param name="handState">state of the hand</param>
/// <param name="handPosition">position of the hand</param>
void CBodyBasics::DrawHand(HandState handState, const D2D1_POINT_2F& handPosition)
{
    D2D1_ELLIPSE ellipse = D2D1::Ellipse(handPosition, c_HandSize, c_HandSize);

    switch (handState)
    {
    case HandState_Closed:
        m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandClosed);
        sendToArduino(CLOSE_GRIPPER);
        break;

    case HandState_Open:
        m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandOpen);
        sendToArduino(OPEN_GRIPPER);
        break;

    case HandState_Lasso:
        m_pRenderTarget->FillEllipse(ellipse, m_pBrushHandLasso);
        break;



    }
}

void CBodyBasics::printConsole(std::string data)
{
    const char* data_cstr = data.c_str();
    size_t size = mbstowcs(NULL, data_cstr, 0) + 1;
    wchar_t* dataConsole = new wchar_t[size];
    mbstowcs(dataConsole, data_cstr, size);

    WriteConsole(console, dataConsole, size, NULL, NULL);
}

bool CBodyBasics::connectArduino(LPCWSTR port)
{
    // Arduino Conection
    serialArduino = CreateFile(port, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (serialArduino == INVALID_HANDLE_VALUE)
    {
        printConsole("Error: Could not open serial port\n");
        return false;
    }
    else
        printConsole("Arduino connected\n");

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(serialArduino, &dcbSerialParams))
    {
        printConsole("Error: Could not obtain serial port parameters\n");
        CloseHandle(serialArduino);
        return false;
    }
    else
        printConsole("Parameters obtained\n");

    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(serialArduino, &dcbSerialParams))
    {
        printConsole("Error: Port parameters could not be configured\n");
        CloseHandle(serialArduino);
        return false;
    }
    else
        printConsole("Configured parameters\n");

    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(serialArduino, &timeouts))
    {
        printConsole("Error: Serial port timeouts could not be configured.\n");
        CloseHandle(serialArduino);
        return false;
    }
    else
        printConsole("Configured timeouts\n");

    return true;
}

void CBodyBasics::sendToArduino(const char command)
{
    char _command = command;
    
    if (WriteFile(serialArduino, &_command, 1, &bytes_written, NULL))
        printConsole("Sent to Arduino [ COMM ] [ " + std::string(&_command, 1) + " ]\n");
    else
        printConsole("Error sending to Arduino [ COMM ] [ " + std::string(&_command, 1) + " ]\n");
    
}

void CBodyBasics::ClearPrevJoints()
{
    for (size_t i = 0; i < m_nPrevJoints; i++)
    {
        m_prevJoints[i].X = m_prevJoints[i].Y = m_prevJoints[i].Z = 0.0f;
        m_setPrevJoints[i] = false;
    }
    m_populatedJoints = false;
    m_currentJoint = 0LL;
}

const int LIMIT_DISTANCE = 500;

const bool CBodyBasics::JointIsMoving(const CameraSpacePoint& pJoint)
{
    if (!m_setPrevJoints[m_nPrevJoints - 1]) return false;
    const CameraSpacePoint centroid = FindCentroid();
    float _distance = euclidianDistance(centroid, pJoint);
    if (_distance > LIMIT_DISTANCE) return true;
    m_prevJoints[GetFarestIdx(pJoint)] = pJoint;
    return false;
}


const CameraSpacePoint CBodyBasics::FindCentroid()
{
    CameraSpacePoint centroid{};

    centroid.X = centroid.Y = centroid.Z = 0.0f;

    for (size_t i = 0; i < m_nPrevJoints; i++)
    {
        centroid.X += m_prevJoints[i].X;
        centroid.Y += m_prevJoints[i].Y;
        centroid.Z += m_prevJoints[i].Z;
    }

    centroid.X /= m_nPrevJoints;
    centroid.Y /= m_nPrevJoints;
    centroid.Z /= m_nPrevJoints;

    cout << "encontrado" << endl;
    return centroid;
}

const INT64 CBodyBasics::GetFarestIdx(const CameraSpacePoint& pJoint)
{
    int idx = -1;
    float _distance = 1e9f;
    for (size_t i = 0; i < m_nPrevJoints; i++)
    {
        if (euclidianDistance(pJoint, m_prevJoints[i]) > _distance)
        {
            _distance = euclidianDistance(pJoint, m_prevJoints[i]);
            idx = i;
        }
    }
    return idx;
}

const bool CBodyBasics::PopulatedPrevJoints()
{
    return m_setPrevJoints[m_nPrevJoints - 1];
}
