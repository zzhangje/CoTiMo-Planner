#include <grpcpp/grpcpp.h>
#include <windows.h>

#include <mutex>
#include <thread>

#include "config.h"
#include "log.hpp"
#include "map.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "visualize.hpp"

using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryState;
using com::nextinnovation::armtrajectoryservice::Response;
using google::protobuf::RepeatedPtrField;
using grpc::Channel;
using grpc::ClientContext;

std::mutex mtx;
RepeatedPtrField<ArmTrajectoryState> trajectory;
bool newTrajectory = false;

class Client {
 public:
  explicit Client(const std::string& addr, const ArmTrajectoryParameter& request)
      : stub_(ArmTrajectoryService::NewStub(
            grpc::CreateChannel(addr, grpc::InsecureChannelCredentials()))) {
    ClientContext context;
    Response response;

    if (auto status = stub_->generate(&context, request, &response); status.ok()) {
      if (response.has_trajectory()) {
        {
          std::unique_lock<std::mutex> lck(mtx);
          trajectory = response.trajectory().states();
          newTrajectory = true;
        }

        // print the trajectory
        for (const ArmTrajectoryState& state : trajectory) {
          log_info("t=%.3f, h=%.2f, θ=%.2f, vel_h=%.2f, vel_θ=%.2f, V_h=%.2f, V_θ=%.2f",
                   state.timestamp(),
                   state.position().shoulderheightmeter(),
                   state.position().elbowpositiondegree(),
                   state.velocity().shouldervelocitymeterpersecond(),
                   state.velocity().elbowvelocitydegreepersecond(),
                   state.voltage().shouldervoltagevolt(),
                   state.voltage().elbowvoltagevolt());
        }
      }
    }
  }

 private:
  std::unique_ptr<ArmTrajectoryService::Stub> stub_;
};

// 窗口过程函数声明
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
  switch (uMsg) {
    case WM_DESTROY:
      PostQuitMessage(0);
      return 0;

    case WM_PAINT: {
      PAINTSTRUCT ps;
      HDC hdc = BeginPaint(hwnd, &ps);

      // 获取设备上下文
      HDC hdcMem = CreateCompatibleDC(hdc);
      HBITMAP hBitmap = CreateCompatibleBitmap(hdc, 200, 200);
      SelectObject(hdcMem, hBitmap);

      // 设置背景颜色
      SetBkColor(hdcMem, RGB(255, 255, 255));
      FillRect(hdcMem, &ps.rcPaint, (HBRUSH)(COLOR_WINDOW + 1));

      // get values from shared memory
      bool newTraj = false;
      RepeatedPtrField<ArmTrajectoryState> traj;
      {
        std::unique_lock<std::mutex> lck(mtx);
        if (newTrajectory) {
          traj = trajectory;
          newTraj = true;
          newTrajectory = false;
        }
      }

      // 将内存设备上下文的内容复制到屏幕设备上下文
      BitBlt(hdc, 0, 0, 200, 200, hdcMem, 0, 0, SRCCOPY);

      // 释放资源
      DeleteObject(hBitmap);
      DeleteDC(hdcMem);

      EndPaint(hwnd, &ps);
      return 0;
    }
  }

  return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

// 主函数
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
  const char CLASS_NAME[] = "Sample Window Class";

  WNDCLASS wc = {};
  wc.lpfnWndProc = WindowProc;
  wc.hInstance = hInstance;
  wc.lpszClassName = CLASS_NAME;

  RegisterClass(&wc);

  HWND hwnd = CreateWindowEx(
      0,                             // 扩展样式
      CLASS_NAME,                    // 窗口类名
      "Cyber Planner 2025",          // 窗口标题
      WS_OVERLAPPEDWINDOW,           // 窗口样式
      CW_USEDEFAULT, CW_USEDEFAULT,  // 窗口位置
      CW_USEDEFAULT, CW_USEDEFAULT,  // 窗口大小
      NULL,                          // 父窗口句柄
      NULL,                          // 菜单句柄
      hInstance,                     // 实例句柄
      NULL                           // 创建参数
  );

  if (hwnd == NULL) {
    return 0;
  }

  ShowWindow(hwnd, nCmdShow);

  auto request = std::make_shared<ArmTrajectoryParameter>();
  request->mutable_start()->set_shoulderheightmeter(0.0);
  request->mutable_start()->set_elbowpositiondegree(0.0);
  request->mutable_end()->set_shoulderheightmeter(0.0);
  request->mutable_end()->set_elbowpositiondegree(90.0);
  request->set_hasalgae(false);
  request->set_hascoral(false);
  Client("localhost:" + config::params::GRPC_PORT, *request);

  MSG msg = {};
  while (GetMessage(&msg, NULL, 0, 0)) {
    TranslateMessage(&msg);
    DispatchMessage(&msg);
  }

  return 0;
}
