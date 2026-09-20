#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#include <SenseGlove/Core/HandLayer.hpp>
#include <SenseGlove/Core/HandPose.hpp>
#include <SenseGlove/Core/HapticGlove.hpp>
#include <SenseGlove/Core/Library.hpp>
#include <SenseGlove/Core/Nova2Glove.hpp>
#include <SenseGlove/Core/Nova2GloveSensorData.hpp>
#include <SenseGlove/Core/Quat.hpp>
#include <SenseGlove/Core/SenseCom.hpp>
#include <SenseGlove/Core/Vect3D.hpp>

#include "nova2_reader_backend.hpp"
#include "nova2_reader_window.hpp"

#include <chrono>
#include <cctype>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace
{

std::mutex g_sdk_mutex;

std::int64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

void append_json_string(std::ostringstream & out, const std::string & value)
{
  out << '"';
  for (const char ch : value) {
    switch (ch) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << ch; break;
    }
  }
  out << '"';
}

void append_vect3(std::ostringstream & out, const SGCore::Kinematics::Vect3D & value)
{
  out << '[' << value.GetX() << ',' << value.GetY() << ',' << value.GetZ() << ']';
}

void append_quat(std::ostringstream & out, const SGCore::Kinematics::Quat & value)
{
  out << '[' << value.GetX() << ',' << value.GetY() << ',' << value.GetZ() << ',' << value.GetW() << ']';
}

void append_float_array(std::ostringstream & out, const std::vector<float> & values)
{
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    out << values[i];
  }
  out << ']';
}

void append_sensor_channels(
  std::ostringstream & out,
  const SGCore::Nova::Nova2GloveSensorData & sensor_data)
{
  // Nova 2 exposes six physical sensor channels: one thumb-flexion sensor,
  // two index-flexion sensors, one middle-flexion sensor, one ring-flexion
  // sensor, and one thumb-abduction sensor. Pinky flexion is derived from
  // the ring-finger sensor; it is not a separate physical sensor channel.
  out << "{\"thumb_flexion\":" << sensor_data.GetSensorValue(
    SGCore::EFinger::Thumb,
    SGCore::Nova::Nova2GloveSensorData::ESensorLocation::FlexionProximal);
  out << ",\"index_flexion_proximal\":" << sensor_data.GetSensorValue(
    SGCore::EFinger::Index,
    SGCore::Nova::Nova2GloveSensorData::ESensorLocation::FlexionProximal);
  out << ",\"index_flexion_distal\":" << sensor_data.GetSensorValue(
    SGCore::EFinger::Index,
    SGCore::Nova::Nova2GloveSensorData::ESensorLocation::FlexionDistal);
  out << ",\"middle_flexion\":" << sensor_data.GetSensorValue(
    SGCore::EFinger::Middle,
    SGCore::Nova::Nova2GloveSensorData::ESensorLocation::FlexionProximal);
  out << ",\"ring_flexion\":" << sensor_data.GetSensorValue(
    SGCore::EFinger::Ring,
    SGCore::Nova::Nova2GloveSensorData::ESensorLocation::FlexionProximal);
  out << ",\"thumb_abduction\":" << sensor_data.GetSensorValue(
    SGCore::EFinger::Thumb,
    SGCore::Nova::Nova2GloveSensorData::ESensorLocation::Abduction);
  out << '}';
}

template<typename T, typename AppendFn>
void append_nested_array(std::ostringstream & out, const std::vector<std::vector<T>> & values, AppendFn append)
{
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ',';
    }
    out << '[';
    for (std::size_t j = 0; j < values[i].size(); ++j) {
      if (j > 0) {
        out << ',';
      }
      append(out, values[i][j]);
    }
    out << ']';
  }
  out << ']';
}

bool append_hand(
  std::ostringstream & out,
  bool right_hand,
  bool index_influence_others,
  bool & first_hand)
{
  std::shared_ptr<SGCore::HapticGlove> hand_glove;
  if (!SGCore::HandLayer::GetGloveInstance(right_hand, hand_glove)) {
    return false;
  }
  const auto nova2 = std::dynamic_pointer_cast<SGCore::Nova::Nova2Glove>(hand_glove);
  if (!nova2) {
    return false;
  }
  // Configure the live HandLayer instance, not a separately retrieved glove copy.
  auto & glove = *nova2;
  glove.SetIndexInfluencesOthers(index_influence_others);
  if (glove.DoesIndexInfluenceOthers() != index_influence_others) {
    return false;
  }

  SGCore::HandPose pose;
  // HandLayer is the SDK's live per-hand pose path and also advances Nova calibration.
  if (!SGCore::HandLayer::GetHandPose(right_hand, pose)) {
    return false;
  }
  const bool applied_index_influence = glove.DoesIndexInfluenceOthers();
  if (applied_index_influence != index_influence_others) {
    return false;
  }

  SGCore::Nova::Nova2GloveSensorData sensor_data;
  const bool sensor_data_valid = glove.GetSensorData(sensor_data);
  SGCore::Kinematics::Quat imu_rotation;
  const bool imu_valid = glove.GetImuRotation(imu_rotation);
  float battery_level = -1.0f;
  const bool battery_valid = glove.GetBatteryLevel(battery_level);

  if (!first_hand) {
    out << ',';
  }
  first_hand = false;

  out << '{';
  out << "\"side\":";
  append_json_string(out, right_hand ? "right" : "left");
  out << ",\"glove_id\":" << (right_hand ? 1 : 0);
  out << ",\"connected\":true";
  out << ",\"index_influence_others\":" << (applied_index_influence ? "true" : "false");
  out << ",\"is_right\":" << (right_hand ? "true" : "false");
  out << ",\"sensor_data_valid\":" << (sensor_data_valid ? "true" : "false");
  out << ",\"sensor_channels\":";
  append_sensor_channels(out, sensor_data);
  out << ",\"imu_orientation_xyzw\":";
  append_quat(out, imu_rotation);
  out << ",\"imu_valid\":" << (imu_valid ? "true" : "false");
  out << ",\"battery_level\":" << (battery_valid ? battery_level : -1.0f);
  out << ",\"battery_valid\":" << (battery_valid ? "true" : "false");
  out << ",\"is_charging\":" << (glove.IsCharging() ? "true" : "false");
  out << ",\"hand_angles_rad\":";
  append_nested_array(out, pose.GetHandAngles(), append_vect3);
  out << ",\"normalized_flexion\":";
  append_float_array(out, pose.GetNormalizedFlexion(true));
  out << ",\"joint_positions_mm\":";
  append_nested_array(out, pose.GetJointPositions(), append_vect3);
  out << ",\"joint_rotations_xyzw\":";
  append_nested_array(out, pose.GetJointRotations(), append_quat);
  out << '}';
  return true;
}

std::string build_packet(const ReaderOptions & options)
{
  std::lock_guard<std::mutex> lock(g_sdk_mutex);
  std::ostringstream out;
  out << "{\"source\":\"senseglove_nova2\",\"version\":3,\"timestamp_ms\":" << now_ms() << ",\"hands\":[";
  bool first_hand = true;
  if (options.left) {
    append_hand(out, false, options.index_influence_others, first_hand);
  }
  if (options.right) {
    append_hand(out, true, options.index_influence_others, first_hand);
  }
  out << "]}";
  return out.str();
}

}  // namespace

// Legacy fixed-coordinate Win32 implementation, retained only as migration
// reference and excluded from the translation unit.
#if 0
namespace
{

constexpr int ID_HOST = 1001;
constexpr int ID_PORT = 1002;
constexpr int ID_HAND_LEFT = 1003;
constexpr int ID_HAND_RIGHT = 1004;
constexpr int ID_HAND_BOTH = 1005;
constexpr int ID_DEVICE_BRAINCO = 1006;
constexpr int ID_DEVICE_YINSI = 1007;
constexpr int ID_START = 1008;
constexpr int ID_STOP = 1009;
constexpr int ID_STATUS_SENSECOM = 1010;
constexpr int ID_STATUS_LEFT = 1011;
constexpr int ID_STATUS_RIGHT = 1012;
constexpr int ID_STATUS_SENDER = 1013;
constexpr int ID_PACKET_PREVIEW = 1014;
constexpr UINT ID_TIMER = 1;

void UdpSender::stop()
{
  running = false;
  if (worker.joinable()) {
    worker.join();
  }
}

UdpSender::~UdpSender()
{
  stop();
}

struct UiState
{
  HWND host{};
  HWND port{};
  HWND hand_left{};
  HWND hand_right{};
  HWND hand_both{};
  HWND brainco{};
  HWND yinsi{};
  HWND start{};
  HWND stop{};
  HWND sensecom_status{};
  HWND left_status{};
  HWND right_status{};
  HWND sender_status{};
  HWND packet_preview{};
  std::string shown_packet;
  HFONT preview_font{};
  unsigned int timer_ticks{};
  Sender sender;
};

std::string get_text(HWND control)
{
  char buffer[512]{};
  GetWindowTextA(control, buffer, static_cast<int>(sizeof(buffer)));
  return buffer;
}

void set_text(HWND control, const std::string & value)
{
  SetWindowTextA(control, value.c_str());
}

HWND make_control(
  const char * class_name,
  const char * text,
  DWORD style,
  int x,
  int y,
  int width,
  int height,
  HWND parent,
  int id)
{
  return CreateWindowExA(
    0,
    class_name,
    text,
    WS_CHILD | WS_VISIBLE | style,
    x,
    y,
    width,
    height,
    parent,
    reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
    GetModuleHandleA(nullptr),
    nullptr);
}

void make_label(HWND parent, const char * text, int x, int y, int width, int height)
{
  make_control("STATIC", text, 0, x, y, width, height, parent, 0);
}

void set_default_font(HWND control)
{
  SendMessageA(control, WM_SETFONT, reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), TRUE);
}

void set_all_fonts(UiState & ui)
{
  const HWND controls[] = {
    ui.host, ui.port, ui.hand_left, ui.hand_right, ui.hand_both,
    ui.brainco, ui.yinsi, ui.start, ui.stop, ui.sensecom_status,
    ui.left_status, ui.right_status, ui.sender_status, ui.packet_preview};
  for (HWND control : controls) {
    set_default_font(control);
  }
  ui.preview_font = CreateFontA(
    -16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
    OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
    FIXED_PITCH | FF_MODERN, "Consolas");
  SendMessageA(ui.packet_preview, WM_SETFONT, reinterpret_cast<WPARAM>(ui.preview_font), TRUE);
}

std::string format_json(const std::string & json)
{
  std::string out;
  out.reserve(json.size() * 2);
  int indent = 0;
  bool in_string = false;
  bool escaped = false;
  const auto newline = [&out, &indent]() {
    out.push_back('\r');
    out.push_back('\n');
    out.append(static_cast<std::size_t>(indent) * 2, ' ');
  };
  for (const char ch : json) {
    if (in_string) {
      out.push_back(ch);
      if (escaped) {
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        in_string = false;
      }
      continue;
    }
    if (ch == '"') {
      in_string = true;
      out.push_back(ch);
    } else if (ch == '{' || ch == '[') {
      out.push_back(ch);
      ++indent;
      newline();
    } else if (ch == '}' || ch == ']') {
      --indent;
      newline();
      out.push_back(ch);
    } else if (ch == ',') {
      out.push_back(ch);
      newline();
    } else if (ch == ':') {
      out.append(": ");
    } else if (!std::isspace(static_cast<unsigned char>(ch))) {
      out.push_back(ch);
    }
  }
  return out;
}

bool read_options(const UiState & ui, Options & options, std::string & error)
{
  options.host = get_text(ui.host);
  if (options.host.empty()) {
    options.host = "127.0.0.1";
  }

  const std::string port_text = get_text(ui.port);
  try {
    options.port = port_text.empty() ? 15020 : std::stoi(port_text);
  } catch (...) {
    error = "Port must be a number.";
    return false;
  }
  if (options.port <= 0 || options.port > 65535) {
    error = "Port must be between 1 and 65535.";
    return false;
  }
  if (IsDlgButtonChecked(GetParent(ui.hand_left), ID_HAND_LEFT) == BST_CHECKED) {
    options.left = true;
    options.right = false;
  } else if (IsDlgButtonChecked(GetParent(ui.hand_right), ID_HAND_RIGHT) == BST_CHECKED) {
    options.left = false;
    options.right = true;
  } else {
    options.left = true;
    options.right = true;
  }
  options.rate_hz = 60.0;
  options.start_com = true;
  options.index_influence_others = false;
  return true;
}

void start_sender(UiState & ui, const Options & options)
{
  ui.sender.stop();
  ui.sender.sent_packets = 0;
  {
    std::lock_guard<std::mutex> lock(ui.sender.packet_mutex);
    ui.sender.latest_packet.clear();
  }
  ui.sender.running = true;
  ui.sender.worker = std::thread([&ui, options]() {
    if (options.start_com && !SGCore::SenseCom::ScanningActive()) {
      SGCore::SenseCom::StartupSenseCom();
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
      ui.sender.running = false;
      return;
    }

    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(static_cast<u_short>(options.port));
    if (inet_pton(AF_INET, options.host.c_str(), &target.sin_addr) != 1) {
      closesocket(sock);
      ui.sender.running = false;
      return;
    }

    const auto period = std::chrono::duration<double>(1.0 / options.rate_hz);
    while (ui.sender.running) {
      const auto loop_start = std::chrono::steady_clock::now();
      const std::string packet = build_packet(options);
      {
        std::lock_guard<std::mutex> lock(ui.sender.packet_mutex);
        ui.sender.latest_packet = packet;
      }
      sendto(
        sock,
        packet.c_str(),
        static_cast<int>(packet.size()),
        0,
        reinterpret_cast<const sockaddr *>(&target),
        sizeof(target));
      ++ui.sender.sent_packets;
      std::this_thread::sleep_until(loop_start + period);
    }
    closesocket(sock);
  });
}

void refresh_packet_preview(UiState & ui)
{
  std::string packet;
  {
    std::lock_guard<std::mutex> lock(ui.sender.packet_mutex);
    packet = ui.sender.latest_packet;
  }
  if (packet.empty()) {
    packet = "Waiting for the first UDP JSON frame...";
  } else {
    packet = format_json(packet);
  }
  if (packet != ui.shown_packet) {
    set_text(ui.packet_preview, packet);
    ui.shown_packet = std::move(packet);
  }
}

void draw_card(HDC dc, const RECT & rect, const char * title, const char * subtitle = nullptr)
{
  HBRUSH fill = CreateSolidBrush(RGB(255, 255, 255));
  HPEN border = CreatePen(PS_SOLID, 1, RGB(220, 222, 227));
  HGDIOBJ old_brush = SelectObject(dc, fill);
  HGDIOBJ old_pen = SelectObject(dc, border);
  RoundRect(dc, rect.left, rect.top, rect.right, rect.bottom, 16, 16);
  SelectObject(dc, old_brush);
  SelectObject(dc, old_pen);
  DeleteObject(fill);
  DeleteObject(border);

  HFONT title_font = CreateFontA(
    -18, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
    OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
    DEFAULT_PITCH | FF_DONTCARE, "Segoe UI");
  HGDIOBJ old_font = SelectObject(dc, title_font);
  SetBkMode(dc, TRANSPARENT);
  SetTextColor(dc, RGB(28, 30, 34));
  RECT title_rect{rect.left + 20, rect.top + 14, rect.right - 20, rect.top + 38};
  DrawTextA(dc, title, -1, &title_rect, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
  SelectObject(dc, old_font);
  DeleteObject(title_font);
  if (subtitle) {
    SetTextColor(dc, RGB(110, 114, 122));
    RECT subtitle_rect{rect.left + 20, rect.top + 40, rect.right - 20, rect.top + 58};
    DrawTextA(dc, subtitle, -1, &subtitle_rect, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
  }
}

void draw_action_button(const DRAWITEMSTRUCT & item, bool primary)
{
  HDC dc = item.hDC;
  RECT rect = item.rcItem;
  const bool enabled = (item.itemState & ODS_DISABLED) == 0;
  const bool pressed = (item.itemState & ODS_SELECTED) != 0;
  const COLORREF color = primary
    ? (pressed ? RGB(0, 95, 205) : RGB(0, 122, 255))
    : (pressed ? RGB(215, 218, 224) : RGB(235, 237, 241));
  HBRUSH fill = CreateSolidBrush(enabled ? color : RGB(210, 213, 219));
  HPEN border = CreatePen(PS_SOLID, 1, enabled ? color : RGB(210, 213, 219));
  HGDIOBJ old_brush = SelectObject(dc, fill);
  HGDIOBJ old_pen = SelectObject(dc, border);
  RoundRect(dc, rect.left, rect.top, rect.right, rect.bottom, 12, 12);
  SelectObject(dc, old_brush);
  SelectObject(dc, old_pen);
  DeleteObject(fill);
  DeleteObject(border);
  char text[64]{};
  GetWindowTextA(item.hwndItem, text, static_cast<int>(sizeof(text)));
  SetBkMode(dc, TRANSPARENT);
  SetTextColor(dc, primary ? RGB(255, 255, 255) : RGB(45, 48, 55));
  DrawTextA(dc, text, -1, &rect, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
}

void refresh_device_status(UiState & ui)
{
  const bool scanning = SGCore::SenseCom::ScanningActive();
  set_text(ui.sensecom_status, scanning ? "SenseCom: detected / scanning" : "SenseCom: not detected");

  std::lock_guard<std::mutex> lock(g_sdk_mutex);
  for (const bool right_hand : {false, true}) {
    SGCore::Nova::Nova2Glove glove;
    HWND status = right_hand ? ui.right_status : ui.left_status;
    if (!SGCore::Nova::Nova2Glove::GetNova2Glove(right_hand, glove)) {
      set_text(status, right_hand ? "Right hand: not connected" : "Left hand: not connected");
      continue;
    }
    float battery = -1.0f;
    const bool battery_valid = glove.GetBatteryLevel(battery);
    std::ostringstream text;
    text << (right_hand ? "Right hand: " : "Left hand: ");
    if (battery_valid && battery >= 0.0f) {
      text << static_cast<int>(battery * 100.0f + 0.5f) << '%';
    } else {
      text << "battery unknown";
    }
    text << (glove.IsCharging() ? ", charging" : ", not charging");
    set_text(status, text.str());
  }
}

LRESULT CALLBACK window_proc(HWND window, UINT message, WPARAM wparam, LPARAM lparam)
{
  auto * ui = reinterpret_cast<UiState *>(GetWindowLongPtrA(window, GWLP_USERDATA));
  if (message == WM_NCCREATE) {
    auto * create = reinterpret_cast<CREATESTRUCTA *>(lparam);
    ui = static_cast<UiState *>(create->lpCreateParams);
    SetWindowLongPtrA(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(ui));
  }

  switch (message) {
    case WM_CREATE:
      ui->host = make_control("EDIT", "127.0.0.1", WS_BORDER | ES_AUTOHSCROLL, 50, 110, 250, 28, window, ID_HOST);
      ui->port = make_control("EDIT", "15020", WS_BORDER | ES_AUTOHSCROLL | ES_NUMBER, 330, 110, 150, 28, window, ID_PORT);
      make_label(window, "Ubuntu IP", 50, 84, 110, 20);
      make_label(window, "UDP port", 330, 84, 100, 20);
      make_label(window, "Hand mode", 600, 84, 100, 20);
      ui->hand_left = make_control("BUTTON", "Left", BS_AUTORADIOBUTTON | WS_GROUP, 600, 110, 90, 28, window, ID_HAND_LEFT);
      ui->hand_right = make_control("BUTTON", "Right", BS_AUTORADIOBUTTON, 700, 110, 90, 28, window, ID_HAND_RIGHT);
      ui->hand_both = make_control("BUTTON", "Both", BS_AUTORADIOBUTTON, 800, 110, 90, 28, window, ID_HAND_BOTH);
      CheckDlgButton(window, ID_HAND_BOTH, BST_CHECKED);
      make_label(window, "Target glove", 920, 84, 100, 20);
      ui->brainco = make_control("BUTTON", "BrainCo", BS_AUTORADIOBUTTON | WS_GROUP, 920, 110, 90, 28, window, ID_DEVICE_BRAINCO);
      ui->yinsi = make_control("BUTTON", "YinShi (reserved)", BS_AUTORADIOBUTTON, 1010, 110, 120, 28, window, ID_DEVICE_YINSI);
      CheckDlgButton(window, ID_DEVICE_BRAINCO, BST_CHECKED);
      ui->sensecom_status = make_control("STATIC", "SenseCom: checking...", 0, 50, 235, 360, 24, window, ID_STATUS_SENSECOM);
      ui->left_status = make_control("STATIC", "Left hand: checking...", 0, 410, 235, 280, 24, window, ID_STATUS_LEFT);
      ui->right_status = make_control("STATIC", "Right hand: checking...", 0, 740, 235, 300, 24, window, ID_STATUS_RIGHT);
      ui->sender_status = make_control("STATIC", "Not sending", 0, 50, 345, 470, 24, window, ID_STATUS_SENDER);
      ui->start = make_control("BUTTON", "Start sending", BS_OWNERDRAW, 810, 327, 150, 42, window, ID_START);
      ui->stop = make_control("BUTTON", "Stop sending", BS_OWNERDRAW, 975, 327, 150, 42, window, ID_STOP);
      ui->packet_preview = make_control(
        "EDIT", "Waiting for the first UDP JSON frame...",
        WS_BORDER | ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL | ES_WANTRETURN | WS_VSCROLL,
        50, 465, 1075, 250, window, ID_PACKET_PREVIEW);
      EnableWindow(ui->stop, FALSE);
      set_all_fonts(*ui);
      SetTimer(window, ID_TIMER, 250, nullptr);
      refresh_device_status(*ui);
      refresh_packet_preview(*ui);
      return 0;

    case WM_TIMER:
      if (wparam == ID_TIMER) {
        if (++ui->timer_ticks % 2 == 0) {
          refresh_device_status(*ui);
        }
        refresh_packet_preview(*ui);
        if (ui->sender.running) {
          set_text(
            ui->sender_status,
            "Sending at 60 Hz UDP JSON | packets: " + std::to_string(ui->sender.sent_packets.load()));
        } else {
          set_text(ui->sender_status, "Not sending");
        }
        EnableWindow(ui->start, ui->sender.running ? FALSE : TRUE);
        EnableWindow(ui->stop, ui->sender.running ? TRUE : FALSE);
      }
      return 0;

    case WM_PAINT: {
      PAINTSTRUCT paint{};
      HDC dc = BeginPaint(window, &paint);
      RECT client{};
      GetClientRect(window, &client);
      HBRUSH background = CreateSolidBrush(RGB(246, 247, 249));
      FillRect(dc, &client, background);
      DeleteObject(background);
      draw_card(dc, RECT{24, 58, 550, 160}, "Network", "Target for the outbound UDP stream");
      draw_card(dc, RECT{566, 58, 1145, 160}, "Control", "Select input hand and robot hand target");
      draw_card(dc, RECT{24, 178, 1145, 280}, "Device status", "Live status retrieved from SenseCom and SGCore");
      draw_card(dc, RECT{24, 298, 1145, 392}, "Sender", "Strongly linked to the live UDP sender");
      draw_card(dc, RECT{24, 410, 1145, 740}, "Latest UDP JSON frame", "Formatted preview of the actual packet sent by the reader");
      HFONT title_font = CreateFontA(
        -28, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
        DEFAULT_PITCH | FF_DONTCARE, "Segoe UI");
      HGDIOBJ old_font = SelectObject(dc, title_font);
      SetBkMode(dc, TRANSPARENT);
      SetTextColor(dc, RGB(25, 28, 34));
      RECT title_rect{28, 16, 800, 48};
      DrawTextA(dc, "Nova 2 UDP Reader", -1, &title_rect, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
      SelectObject(dc, old_font);
      DeleteObject(title_font);
      EndPaint(window, &paint);
      return 0;
    }

    case WM_CTLCOLORSTATIC:
      SetBkMode(reinterpret_cast<HDC>(wparam), TRANSPARENT);
      SetTextColor(reinterpret_cast<HDC>(wparam), RGB(52, 56, 64));
      return reinterpret_cast<LRESULT>(GetStockObject(NULL_BRUSH));

    case WM_CTLCOLOREDIT:
      SetBkColor(reinterpret_cast<HDC>(wparam), RGB(255, 255, 255));
      SetTextColor(reinterpret_cast<HDC>(wparam), RGB(30, 33, 39));
      return reinterpret_cast<LRESULT>(GetStockObject(WHITE_BRUSH));

    case WM_DRAWITEM:
      if (wparam == ID_START || wparam == ID_STOP) {
        draw_action_button(*reinterpret_cast<DRAWITEMSTRUCT *>(lparam), wparam == ID_START);
        return TRUE;
      }
      break;

    case WM_COMMAND:
      if (LOWORD(wparam) == ID_START && HIWORD(wparam) == BN_CLICKED) {
        if (IsDlgButtonChecked(window, ID_DEVICE_YINSI) == BST_CHECKED) {
          MessageBoxA(window, "YinShi sending is not implemented yet.", "Information", MB_OK | MB_ICONINFORMATION);
          return 0;
        }
        Options options;
        std::string error;
        if (!read_options(*ui, options, error)) {
          MessageBoxA(window, error.c_str(), "Invalid parameters", MB_OK | MB_ICONERROR);
          return 0;
        }
        if (!options.left && !options.right) {
          MessageBoxA(window, "Select Left, Right, or Both.", "Invalid parameters", MB_OK | MB_ICONERROR);
          return 0;
        }
        sockaddr_in check{};
        if (inet_pton(AF_INET, options.host.c_str(), &check.sin_addr) != 1) {
          MessageBoxA(window, "IP must be an IPv4 address, for example 192.168.1.20.", "Invalid parameters", MB_OK | MB_ICONERROR);
          return 0;
        }
        start_sender(*ui, options);
        set_text(ui->sender_status, "Starting sender...");
        return 0;
      }
      if (LOWORD(wparam) == ID_STOP && HIWORD(wparam) == BN_CLICKED) {
        ui->sender.stop();
        set_text(ui->sender_status, "Sending stopped");
        return 0;
      }
      return 0;

    case WM_DESTROY:
      KillTimer(window, ID_TIMER);
      ui->sender.stop();
      DeleteObject(ui->preview_font);
      PostQuitMessage(0);
      return 0;
  }
  return DefWindowProcA(window, message, wparam, lparam);
}

}  // namespace

#endif

void UdpSender::stop()
{
  running = false;
  if (worker.joinable()) {
    worker.join();
  }
}

UdpSender::~UdpSender()
{
  stop();
}

std::string format_json(const std::string & json)
{
  std::string out;
  out.reserve(json.size() * 2);
  int indent = 0;
  bool in_string = false;
  bool escaped = false;
  const auto newline = [&out, &indent]() {
    out.append("\r\n");
    out.append(static_cast<std::size_t>(indent) * 2, ' ');
  };
  for (const char ch : json) {
    if (in_string) {
      out.push_back(ch);
      if (escaped) {
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        in_string = false;
      }
    } else if (ch == '"') {
      in_string = true;
      out.push_back(ch);
    } else if (ch == '{' || ch == '[') {
      out.push_back(ch);
      ++indent;
      newline();
    } else if (ch == '}' || ch == ']') {
      --indent;
      newline();
      out.push_back(ch);
    } else if (ch == ',') {
      out.push_back(ch);
      newline();
    } else if (ch == ':') {
      out.append(": ");
    } else if (!std::isspace(static_cast<unsigned char>(ch))) {
      out.push_back(ch);
    }
  }
  return out;
}

void start_sender(UdpSender & sender, const ReaderOptions & options)
{
  sender.stop();
  sender.sent_packets = 0;
  {
    std::lock_guard<std::mutex> lock(sender.packet_mutex);
    sender.latest_packet.clear();
  }
  sender.running = true;
  sender.worker = std::thread([&sender, options]() {
    if (options.start_com && !SGCore::SenseCom::ScanningActive()) {
      SGCore::SenseCom::StartupSenseCom();
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
      sender.running = false;
      return;
    }
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(static_cast<u_short>(options.port));
    if (inet_pton(AF_INET, options.host.c_str(), &target.sin_addr) != 1) {
      closesocket(sock);
      sender.running = false;
      return;
    }

    const auto period = std::chrono::duration<double>(1.0 / options.rate_hz);
    while (sender.running) {
      const auto loop_start = std::chrono::steady_clock::now();
      const std::string packet = build_packet(options);
      {
        std::lock_guard<std::mutex> lock(sender.packet_mutex);
        sender.latest_packet = packet;
      }
      sendto(sock, packet.c_str(), static_cast<int>(packet.size()), 0,
        reinterpret_cast<const sockaddr *>(&target), sizeof(target));
      ++sender.sent_packets;
      std::this_thread::sleep_until(loop_start + period);
    }
    closesocket(sock);
  });
}

ReaderDeviceStatus read_device_status()
{
  ReaderDeviceStatus result;
  result.sensecom_detected = SGCore::SenseCom::ScanningActive();
  result.sensecom = result.sensecom_detected
    ? "SenseCom: detected / scanning" : "SenseCom: not detected";
  std::lock_guard<std::mutex> lock(g_sdk_mutex);
  for (const bool right_hand : {false, true}) {
    SGCore::Nova::Nova2Glove glove;
    std::string & status = right_hand ? result.right : result.left;
    const char * const name = right_hand ? "Right hand" : "Left hand";
    if (!SGCore::Nova::Nova2Glove::GetNova2Glove(right_hand, glove)) {
      status = std::string(name) + ": not connected";
      (right_hand ? result.right_calibration : result.left_calibration) =
        std::string(name) + " calibration: unavailable";
      continue;
    }
    if (right_hand) {
      result.right_connected = true;
    } else {
      result.left_connected = true;
    }
    float battery = -1.0f;
    std::ostringstream text;
    text << name << ": ";
    if (glove.GetBatteryLevel(battery) && battery >= 0.0f) {
      text << static_cast<int>(battery * 100.0f + 0.5f) << '%';
    } else {
      text << "battery unknown";
    }
    text << (glove.IsCharging() ? ", charging" : ", not charging");
    status = text.str();
    (right_hand ? result.right_calibration : result.left_calibration) =
      std::string(name) + " calibration: " +
      SGCore::HapticGlove::ToString(glove.GetCalibrationState());
  }
  return result;
}

#if 0
class ReaderWindow final : public QMainWindow
{
public:
  ReaderWindow()
  {
    setWindowTitle("Nova 2 UDP Reader");
    resize(1240, 900);
    setMinimumSize(980, 720);

    auto * central = new QWidget(this);
    auto * root = new QVBoxLayout(central);
    root->setContentsMargins(28, 24, 28, 28);
    root->setSpacing(18);
    setCentralWidget(central);

    auto * title = new QLabel("Nova 2 UDP Reader", central);
    title->setObjectName("windowTitle");
    auto * subtitle = new QLabel("SenseGlove data to UDP, with a live packet preview.", central);
    subtitle->setObjectName("windowSubtitle");
    root->addWidget(title);
    root->addWidget(subtitle);

    auto * top_row = new QHBoxLayout;
    top_row->setSpacing(18);
    auto * network = make_card("Network", "Outbound UDP destination", &network_layout_);
    auto * network_form = new QFormLayout;
    network_form->setHorizontalSpacing(14);
    network_form->setVerticalSpacing(10);
    host_ = new QLineEdit("127.0.0.1", network);
    port_ = new QLineEdit("15020", network);
    port_->setMaximumWidth(160);
    network_form->addRow("Ubuntu IP", host_);
    network_form->addRow("UDP port", port_);
    network_layout_->addLayout(network_form);
    network_layout_->addStretch();

    auto * control = make_card("Control", "Select hand mode and robot target", &control_layout_);
    auto * control_grid = new QGridLayout;
    control_grid->setHorizontalSpacing(12);
    control_grid->setVerticalSpacing(10);
    hand_left_ = new QRadioButton("Left", control);
    hand_right_ = new QRadioButton("Right", control);
    hand_both_ = new QRadioButton("Both", control);
    hand_both_->setChecked(true);
    brainco_ = new QRadioButton("BrainCo", control);
    yinsi_ = new QRadioButton("YinShi (reserved)", control);
    brainco_->setChecked(true);
    hand_left_->setAutoExclusive(false);
    hand_right_->setAutoExclusive(false);
    hand_both_->setAutoExclusive(false);
    brainco_->setAutoExclusive(false);
    yinsi_->setAutoExclusive(false);
    hand_group_ = new QButtonGroup(this);
    hand_group_->addButton(hand_left_);
    hand_group_->addButton(hand_right_);
    hand_group_->addButton(hand_both_);
    target_group_ = new QButtonGroup(this);
    target_group_->addButton(brainco_);
    target_group_->addButton(yinsi_);
    control_grid->addWidget(new QLabel("Hand mode", control), 0, 0);
    control_grid->addWidget(hand_left_, 0, 1);
    control_grid->addWidget(hand_right_, 0, 2);
    control_grid->addWidget(hand_both_, 0, 3);
    control_grid->addWidget(new QLabel("Target glove", control), 1, 0);
    control_grid->addWidget(brainco_, 1, 1, 1, 2);
    control_grid->addWidget(yinsi_, 1, 3);
    control_layout_->addLayout(control_grid);
    control_layout_->addStretch();
    top_row->addWidget(network, 1);
    top_row->addWidget(control, 2);
    root->addLayout(top_row);

    auto * status = make_card("Device status", "Live status retrieved from SenseCom and SGCore", &status_layout_);
    auto * status_grid = new QGridLayout;
    status_grid->setHorizontalSpacing(18);
    status_grid->setVerticalSpacing(8);
    sensecom_status_ = status_label("SenseCom: checking...", status);
    left_status_ = status_label("Left hand: checking...", status);
    right_status_ = status_label("Right hand: checking...", status);
    status_grid->addWidget(sensecom_status_, 0, 0);
    status_grid->addWidget(left_status_, 0, 1);
    status_grid->addWidget(right_status_, 0, 2);
    status_layout_->addLayout(status_grid);
    root->addWidget(status);

    auto * sender = make_card("Sender", "The preview below is the exact JSON sent by this process", &sender_layout_);
    auto * sender_row = new QHBoxLayout;
    sender_status_ = new QLabel("Not sending", sender);
    sender_status_->setObjectName("senderStatus");
    start_button_ = new QPushButton("Start sending", sender);
    start_button_->setObjectName("primaryButton");
    stop_button_ = new QPushButton("Stop sending", sender);
    stop_button_->setEnabled(false);
    sender_row->addWidget(sender_status_, 1);
    sender_row->addWidget(start_button_);
    sender_row->addWidget(stop_button_);
    sender_layout_->addLayout(sender_row);
    root->addWidget(sender);

    auto * preview = make_card("Latest UDP JSON frame", "Formatted for display; whitespace is the only change", &preview_layout_);
    packet_preview_ = new QPlainTextEdit(preview);
    packet_preview_->setReadOnly(true);
    packet_preview_->setPlainText("Waiting for the first UDP JSON frame...");
    QFont preview_font("Consolas");
    preview_font.setStyleHint(QFont::Monospace);
    packet_preview_->setFont(preview_font);
    preview_layout_->addWidget(packet_preview_);
    root->addWidget(preview, 1);

    connect(start_button_, &QPushButton::clicked, this, [this] { start_sending(); });
    connect(stop_button_, &QPushButton::clicked, this, [this] { stop_sending(); });
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this] { refresh(); });
    timer_->start(200);
    refresh_device_status();
    apply_style();
  }

  ~ReaderWindow() override
  {
    sender_.stop();
  }

private:
  static QFrame * make_card(const char * title, const char * subtitle, QVBoxLayout ** body)
  {
    auto * card = new QFrame;
    card->setObjectName("card");
    auto * layout = new QVBoxLayout(card);
    layout->setContentsMargins(20, 17, 20, 18);
    layout->setSpacing(6);
    auto * card_title = new QLabel(title, card);
    card_title->setObjectName("cardTitle");
    auto * card_subtitle = new QLabel(subtitle, card);
    card_subtitle->setObjectName("cardSubtitle");
    card_subtitle->setWordWrap(true);
    layout->addWidget(card_title);
    layout->addWidget(card_subtitle);
    layout->addSpacing(6);
    *body = layout;
    return card;
  }

  static QLabel * status_label(const char * text, QWidget * parent)
  {
    auto * label = new QLabel(text, parent);
    label->setObjectName("statusLabel");
    label->setMinimumHeight(34);
    return label;
  }

  void apply_style()
  {
    setStyleSheet(
      "QWidget { background: #F5F5F7; color: #1D1D1F; font-family: 'Segoe UI'; font-size: 14px; }"
      "QFrame#card { background: #FFFFFF; border: 1px solid #E5E5EA; border-radius: 16px; }"
      "QLabel#windowTitle { font-size: 28px; font-weight: 600; }"
      "QLabel#windowSubtitle, QLabel#cardSubtitle { color: #6E6E73; }"
      "QLabel#cardTitle { font-size: 16px; font-weight: 600; }"
      "QLabel#statusLabel { background: #F5F5F7; border-radius: 8px; padding: 8px 10px; }"
      "QLabel#senderStatus { font-weight: 600; color: #007AFF; }"
      "QLineEdit, QPlainTextEdit { background: #FBFBFD; border: 1px solid #D1D1D6; border-radius: 8px; padding: 7px; }"
      "QLineEdit:focus, QPlainTextEdit:focus { border: 1px solid #007AFF; }"
      "QPushButton { background: #E9E9ED; border: 0; border-radius: 9px; padding: 9px 16px; min-width: 110px; }"
      "QPushButton:hover { background: #DCDCE1; }"
      "QPushButton#primaryButton { background: #007AFF; color: white; font-weight: 600; }"
      "QPushButton#primaryButton:hover { background: #0071E3; }"
      "QPushButton:disabled { background: #E5E5EA; color: #8E8E93; }");
  }

  void start_sending()
  {
    if (yinsi_->isChecked()) {
      QMessageBox::information(this, "Information", "YinShi sending is not implemented yet.");
      return;
    }
    bool valid_port = false;
    const int port = port_->text().toInt(&valid_port);
    if (!valid_port || port <= 0 || port > 65535) {
      QMessageBox::warning(this, "Invalid parameters", "Port must be between 1 and 65535.");
      return;
    }
    const std::string host = host_->text().trimmed().isEmpty()
      ? "127.0.0.1" : host_->text().trimmed().toStdString();
    sockaddr_in address{};
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
      QMessageBox::warning(this, "Invalid parameters", "IP must be an IPv4 address, for example 192.168.1.20.");
      return;
    }
    Options options;
    options.host = host;
    options.port = port;
    options.left = hand_left_->isChecked() || hand_both_->isChecked();
    options.right = hand_right_->isChecked() || hand_both_->isChecked();
    start_sender(sender_, options);
    sender_status_->setText("Starting sender...");
  }

  void stop_sending()
  {
    sender_.stop();
    sender_status_->setText("Sending stopped");
  }

  void refresh_device_status()
  {
    sensecom_status_->setText(SGCore::SenseCom::ScanningActive()
      ? "SenseCom: detected / scanning" : "SenseCom: not detected");
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    for (const bool right_hand : {false, true}) {
      SGCore::Nova::Nova2Glove glove;
      QLabel * const status = right_hand ? right_status_ : left_status_;
      const char * const name = right_hand ? "Right hand" : "Left hand";
      if (!SGCore::Nova::Nova2Glove::GetNova2Glove(right_hand, glove)) {
        status->setText(QString::fromLatin1(name) + ": not connected");
        continue;
      }
      float battery = -1.0f;
      std::ostringstream text;
      text << name << ": ";
      if (glove.GetBatteryLevel(battery) && battery >= 0.0f) {
        text << static_cast<int>(battery * 100.0f + 0.5f) << '%';
      } else {
        text << "battery unknown";
      }
      text << (glove.IsCharging() ? ", charging" : ", not charging");
      status->setText(QString::fromStdString(text.str()));
    }
  }

  void refresh_packet_preview()
  {
    std::string packet;
    {
      std::lock_guard<std::mutex> lock(sender_.packet_mutex);
      packet = sender_.latest_packet;
    }
    if (packet.empty()) {
      return;
    }
    const QString formatted = QString::fromStdString(format_json(packet));
    if (formatted != shown_packet_) {
      packet_preview_->setPlainText(formatted);
      shown_packet_ = formatted;
    }
  }

  void refresh()
  {
    refresh_packet_preview();
    if (++refresh_ticks_ % 3 == 0) {
      refresh_device_status();
    }
    if (sender_.running) {
      sender_status_->setText(QString::fromLatin1("Sending at 60 Hz UDP JSON | packets: ")
        + QString::number(sender_.sent_packets.load()));
    } else if (start_button_->isEnabled()) {
      sender_status_->setText("Not sending");
    }
    start_button_->setEnabled(!sender_.running);
    stop_button_->setEnabled(sender_.running);
  }

  Sender sender_;
  QLineEdit * host_{};
  QLineEdit * port_{};
  QRadioButton * hand_left_{};
  QRadioButton * hand_right_{};
  QRadioButton * hand_both_{};
  QRadioButton * brainco_{};
  QRadioButton * yinsi_{};
  QButtonGroup * hand_group_{};
  QButtonGroup * target_group_{};
  QLabel * sensecom_status_{};
  QLabel * left_status_{};
  QLabel * right_status_{};
  QLabel * sender_status_{};
  QPushButton * start_button_{};
  QPushButton * stop_button_{};
  QPlainTextEdit * packet_preview_{};
  QTimer * timer_{};
  QVBoxLayout * network_layout_{};
  QVBoxLayout * control_layout_{};
  QVBoxLayout * status_layout_{};
  QVBoxLayout * sender_layout_{};
  QVBoxLayout * preview_layout_{};
  QString shown_packet_;
  unsigned int refresh_ticks_{};
};

}  // namespace

#endif

int main(int argc, char ** argv)
{
  // Hardware self-check: prime the live instance ON, then exercise the real
  // packet path and verify it switches that same HandLayer instance OFF.
  if (argc == 2 && std::string(argv[1]) == "--self-check") {
    int checked = 0;
    for (const bool right_hand : {false, true}) {
      std::shared_ptr<SGCore::HapticGlove> base;
      if (!SGCore::HandLayer::GetGloveInstance(right_hand, base)) {
        continue;
      }
      const auto glove = std::dynamic_pointer_cast<SGCore::Nova::Nova2Glove>(base);
      if (!glove) {
        continue;
      }
      const bool original = glove->DoesIndexInfluenceOthers();
      glove->SetIndexInfluencesOthers(true);
      const bool primed = glove->DoesIndexInfluenceOthers();
      std::ostringstream packet;
      bool first_hand = true;
      const bool emitted = append_hand(packet, right_hand, false, first_hand);
      std::shared_ptr<SGCore::HapticGlove> live_base;
      SGCore::HandLayer::GetGloveInstance(right_hand, live_base);
      const auto live = std::dynamic_pointer_cast<SGCore::Nova::Nova2Glove>(live_base);
      const bool passed = primed && emitted && live && !live->DoesIndexInfluenceOthers()
        && packet.str().find("\"index_influence_others\":false") != std::string::npos;
      glove->SetIndexInfluencesOthers(original);
      std::cout << (right_hand ? "right" : "left") << ": "
                << (passed ? "PASS" : "FAIL") << std::endl;
      if (!passed) {
        return 1;
      }
      ++checked;
    }
    if (!checked) {
      std::cerr << "No connected Nova2 glove; start SenseCom and connect a glove." << std::endl;
      return 2;
    }
    return 0;
  }
  WSADATA wsa_data;
  if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
    return 1;
  }
  const int result = run_reader_window(argc, argv);
  WSACleanup();
  return result;
}
