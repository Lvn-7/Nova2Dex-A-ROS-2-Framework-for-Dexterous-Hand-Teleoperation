#include "nova2_reader_window.hpp"

#include "nova2_reader_backend.hpp"

#include <QApplication>
#include <QButtonGroup>
#include <QByteArray>
#include <QFormLayout>
#include <QFrame>
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QStackedWidget>
#include <QStyle>
#include <QStringList>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

namespace
{

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

    auto * header = new QHBoxLayout;
    auto * title_box = new QVBoxLayout;
    auto * title = new QLabel("Nova 2 UDP Reader", central);
    title->setObjectName("windowTitle");
    auto * subtitle = new QLabel("SenseGlove data bridge", central);
    subtitle->setObjectName("windowSubtitle");
    title_box->addWidget(title);
    title_box->addWidget(subtitle);
    header->addLayout(title_box, 1);
    main_tab_ = navigation_button("Main", 0, central);
    data_tab_ = navigation_button("Data Visual", 1, central);
    format_tab_ = navigation_button("Format", 2, central);
    header->addWidget(main_tab_);
    header->addWidget(data_tab_);
    header->addWidget(format_tab_);
    root->addLayout(header);

    pages_ = new QStackedWidget(central);
    pages_->addWidget(create_main_page());
    pages_->addWidget(create_data_page());
    pages_->addWidget(create_format_page());
    root->addWidget(pages_, 1);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this] { refresh(); });
    timer_->start(200);
    refresh_device_status();
    set_page(0);
    apply_style();
    set_data_hand(false);
  }

  ~ReaderWindow() override
  {
    sender_.stop();
  }

private:
  QPushButton * navigation_button(const char * text, int page, QWidget * parent)
  {
    auto * button = new QPushButton(text, parent);
    button->setObjectName("navigationButton");
    connect(button, &QPushButton::clicked, this, [this, page] { set_page(page); });
    return button;
  }

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

  QWidget * create_main_page()
  {
    auto * page = new QWidget;
    auto * layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(18);

    auto * top_row = new QHBoxLayout;
    top_row->setSpacing(18);
    QVBoxLayout * network_layout = nullptr;
    auto * network = make_card("Network", "Outbound UDP destination", &network_layout);
    auto * network_form = new QFormLayout;
    network_form->setHorizontalSpacing(14);
    network_form->setVerticalSpacing(10);
    host_ = new QLineEdit("127.0.0.1", network);
    port_ = new QLineEdit("15020", network);
    rate_ = new QLineEdit("60", network);
    port_->setMaximumWidth(160);
    rate_->setMaximumWidth(160);
    network_form->addRow("Ubuntu IP", host_);
    network_form->addRow("UDP port", port_);
    network_form->addRow("Send rate (Hz)", rate_);
    network_layout->addLayout(network_form);
    network_layout->addStretch();

    QVBoxLayout * hand_layout = nullptr;
    auto * hand = make_card("Hand mode", "Select the data source to send", &hand_layout);
    auto * hand_row = new QHBoxLayout;
    hand_left_ = new QRadioButton("Left", hand);
    hand_right_ = new QRadioButton("Right", hand);
    hand_both_ = new QRadioButton("Both", hand);
    hand_both_->setChecked(true);
    hand_left_->setAutoExclusive(false);
    hand_right_->setAutoExclusive(false);
    hand_both_->setAutoExclusive(false);
    hand_group_ = new QButtonGroup(this);
    hand_group_->addButton(hand_left_);
    hand_group_->addButton(hand_right_);
    hand_group_->addButton(hand_both_);
    hand_row->addWidget(hand_left_);
    hand_row->addWidget(hand_right_);
    hand_row->addWidget(hand_both_);
    hand_row->addStretch();
    hand_layout->addLayout(hand_row);
    hand_layout->addStretch();
    top_row->addWidget(network, 1);
    top_row->addWidget(hand, 1);
    layout->addLayout(top_row);

    QVBoxLayout * status_layout = nullptr;
    auto * status = make_card("Device status", "Live status retrieved from SenseCom and SGCore", &status_layout);
    auto * status_grid = new QGridLayout;
    status_grid->setHorizontalSpacing(18);
    sensecom_status_ = status_label("SenseCom: checking...", status);
    left_status_ = status_label("Left hand: checking...", status);
    right_status_ = status_label("Right hand: checking...", status);
    left_calibration_status_ = status_label("Left calibration: checking...", status);
    right_calibration_status_ = status_label("Right calibration: checking...", status);
    status_grid->addWidget(sensecom_status_, 0, 0);
    status_grid->addWidget(left_status_, 0, 1);
    status_grid->addWidget(right_status_, 0, 2);
    status_grid->addWidget(left_calibration_status_, 1, 1);
    status_grid->addWidget(right_calibration_status_, 1, 2);
    status_layout->addLayout(status_grid);
    layout->addWidget(status);

    QVBoxLayout * sender_layout = nullptr;
    auto * sender = make_card("Sender", "Start or stop the live UDP stream", &sender_layout);
    auto * sender_row = new QHBoxLayout;
    sending_indicator_ = new QLabel("Not sending", sender);
    sending_indicator_->setObjectName("sendingIndicator");
    start_button_ = new QPushButton("Start sending", sender);
    start_button_->setObjectName("primaryButton");
    stop_button_ = new QPushButton("Stop sending", sender);
    stop_button_->setEnabled(false);
    sender_row->addWidget(sending_indicator_, 1);
    sender_row->addWidget(start_button_);
    sender_row->addWidget(stop_button_);
    sender_layout->addLayout(sender_row);
    layout->addWidget(sender);
    layout->addStretch();

    auto * preset_row = new QHBoxLayout;
    auto * preset1_button = new QPushButton(QStringLiteral("预设1（L）"), page);
    auto * preset2_button = new QPushButton(QStringLiteral("预设2（W）"), page);
    preset_row->addWidget(preset1_button);
    preset_row->addWidget(preset2_button);
    preset_row->addStretch();
    layout->addLayout(preset_row);

    connect(start_button_, &QPushButton::clicked, this, [this] { start_sending(); });
    connect(stop_button_, &QPushButton::clicked, this, [this] { sender_.stop(); });
    connect(preset1_button, &QPushButton::clicked, this, [this] {
      start_preset("192.168.1.103", 15020);
    });
    connect(preset2_button, &QPushButton::clicked, this, [this] {
      start_preset("192.168.123.211", 6000);
    });
    return page;
  }

  QWidget * create_data_page()
  {
    auto * page = new QWidget;
    auto * page_layout = new QVBoxLayout(page);
    page_layout->setContentsMargins(0, 0, 0, 0);
    page_layout->setSpacing(12);
    auto * selector = new QHBoxLayout;
    data_left_button_ = new QPushButton("Left hand", page);
    data_right_button_ = new QPushButton("Right hand", page);
    data_left_button_->setObjectName("handViewButton");
    data_right_button_->setObjectName("handViewButton");
    selector->addWidget(data_left_button_);
    selector->addWidget(data_right_button_);
    selector->addStretch();
    page_layout->addLayout(selector);

    QVBoxLayout * layout = nullptr;
    auto * card = make_card("Data Visual", "Latest selected-hand data from the UDP payload", &layout);
    data_tree_ = new QTreeWidget(card);
    data_tree_->setColumnCount(5);
    data_tree_->setHeaderLabels({"Finger", "Joint / node [0]", "[1]", "[2]", "[3]"});
    data_tree_->setAlternatingRowColors(true);
    data_tree_->setRootIsDecorated(false);
    data_tree_->setItemsExpandable(false);
    data_tree_->setExpandsOnDoubleClick(false);
    data_tree_->setIndentation(0);
    data_tree_->setUniformRowHeights(true);
    data_tree_->header()->setStretchLastSection(true);
    data_tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    data_tree_->header()->setMinimumSectionSize(180);
    data_tree_->setColumnWidth(0, 260);
    data_tree_->setColumnWidth(1, 240);
    data_tree_->setColumnWidth(2, 240);
    data_tree_->setColumnWidth(3, 240);
    data_tree_->setColumnWidth(4, 240);
    add_placeholder("Waiting for the first UDP JSON frame...");
    layout->addWidget(data_tree_, 1);
    page_layout->addWidget(card, 1);
    connect(data_left_button_, &QPushButton::clicked, this, [this] { set_data_hand(false); });
    connect(data_right_button_, &QPushButton::clicked, this, [this] { set_data_hand(true); });
    return page;
  }

  QWidget * create_format_page()
  {
    QVBoxLayout * layout = nullptr;
    auto * page = make_card("Format", "UDP payload definition, version 3", &layout);
    auto * format = new QTreeWidget(page);
    format->setColumnCount(4);
    format->setHeaderLabels({"Field", "Type / shape", "Definition and order", "SDK source"});
    format->setAlternatingRowColors(true);
    format->setRootIsDecorated(false);
    format->setUniformRowHeights(true);
    format->header()->setStretchLastSection(true);
    format->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    format->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    format->setColumnWidth(0, 250);
    format->setColumnWidth(1, 180);
    add_format_row(format, "source", "string", "Always senseglove_nova2", "Reader constant");
    add_format_row(format, "version", "integer", "Always 3", "Reader constant");
    add_format_row(format, "timestamp_ms", "integer", "steady_clock milliseconds; compare intervals only, not Unix time", "Reader steady_clock");
    add_format_row(format, "hands", "array[0..2]", "Selected frames; emission order is left, then right", "Reader selection");
    add_format_row(format, "hands[].side", "string", "left or right; use this instead of array index", "Reader hand request");
    add_format_row(format, "hands[].glove_id", "integer", "0 = left, 1 = right", "Reader constant");
    add_format_row(format, "hands[].connected", "bool", "true only when SDK returned this glove pose", "HandLayer::GetGloveInstance / GetHandPose");
    add_format_row(format, "hands[].index_influence_others", "bool", "Actual index coupling setting; false by default", "Nova2Glove::DoesIndexInfluenceOthers");
    add_format_row(format, "hands[].is_right", "bool", "true for right, false for left", "Reader hand request");
    add_format_row(format, "hands[].sensor_data_valid", "bool", "Whether sensor data read succeeded", "Nova2Glove::GetSensorData");
    add_format_row(format, "sensor_channels", "object / 6 scalars", "Named fields; object order is not semantic", "Nova2GloveSensorData::GetSensorValue");
    add_format_row(format, "  thumb_flexion", "scalar", "Thumb / FlexionProximal", "GetSensorValue(Thumb, FlexionProximal)");
    add_format_row(format, "  index_flexion_proximal", "scalar", "Index / FlexionProximal", "GetSensorValue(Index, FlexionProximal)");
    add_format_row(format, "  index_flexion_distal", "scalar", "Index / FlexionDistal", "GetSensorValue(Index, FlexionDistal)");
    add_format_row(format, "  middle_flexion", "scalar", "Middle / FlexionProximal", "GetSensorValue(Middle, FlexionProximal)");
    add_format_row(format, "  ring_flexion", "scalar", "Ring / FlexionProximal; pinky is SDK-derived", "GetSensorValue(Ring, FlexionProximal)");
    add_format_row(format, "  thumb_abduction", "scalar", "Thumb / Abduction", "GetSensorValue(Thumb, Abduction)");
    add_format_row(format, "imu_orientation_xyzw", "float[4]", "Quaternion order [x, y, z, w]; orientation only", "Nova2Glove::GetImuRotation");
    add_format_row(format, "imu_valid", "bool", "Whether IMU read succeeded", "Nova2Glove::GetImuRotation");
    add_format_row(format, "battery_level", "float", "0..1 when valid; -1 when unavailable", "Nova2Glove::GetBatteryLevel");
    add_format_row(format, "battery_valid", "bool", "Whether battery read succeeded", "Nova2Glove::GetBatteryLevel");
    add_format_row(format, "is_charging", "bool", "Current charging state", "Nova2Glove::IsCharging");
    add_format_row(format, "hand_angles_rad", "float[5][3][3]", "finger -> joint -> Euler XYZ; finger order thumb, index, middle, ring, pinky", "HandLayer::GetHandPose -> HandPose::GetHandAngles");
    add_format_row(format, "normalized_flexion", "float[5]", "0..1; finger order thumb, index, middle, ring, pinky", "HandLayer::GetHandPose -> HandPose::GetNormalizedFlexion(true)");
    add_format_row(format, "joint_positions_mm", "float[5][4][3]", "finger -> node -> XYZ, in mm; SDK node order retained", "HandLayer::GetHandPose -> HandPose::GetJointPositions");
    add_format_row(format, "joint_rotations_xyzw", "float[5][4][4]", "finger -> node -> quaternion [x, y, z, w]; SDK node order retained", "HandLayer::GetHandPose -> HandPose::GetJointRotations");
    layout->addWidget(format, 1);
    return page;
  }

  static void add_format_row(
    QTreeWidget * table,
    const char * field,
    const char * type,
    const char * definition,
    const char * source)
  {
    auto * row = new QTreeWidgetItem(table, {field, type, definition, source});
    row->setToolTip(2, definition);
    row->setToolTip(3, source);
  }

  void set_page(int page)
  {
    pages_->setCurrentIndex(page);
    main_tab_->setProperty("selected", page == 0);
    data_tab_->setProperty("selected", page == 1);
    format_tab_->setProperty("selected", page == 2);
    for (QPushButton * button : {main_tab_, data_tab_, format_tab_}) {
      button->style()->unpolish(button);
      button->style()->polish(button);
    }
  }

  void set_data_hand(bool right_hand)
  {
    data_right_hand_ = right_hand;
    data_left_button_->setProperty("selected", !right_hand);
    data_right_button_->setProperty("selected", right_hand);
    for (QPushButton * button : {data_left_button_, data_right_button_}) {
      button->style()->unpolish(button);
      button->style()->polish(button);
    }
    shown_packet_.clear();
    add_placeholder(right_hand ? "Waiting for a right-hand frame..." : "Waiting for a left-hand frame...");
    refresh_packet_preview();
  }

  static std::string selected_hand_json(const std::string & packet, bool right_hand)
  {
    const std::string side = right_hand ? "\"side\":\"right\"" : "\"side\":\"left\"";
    bool in_string = false;
    bool escaped = false;
    int depth = 0;
    std::size_t object_start = std::string::npos;
    for (std::size_t i = 0; i < packet.size(); ++i) {
      const char ch = packet[i];
      if (in_string) {
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
      } else if (ch == '{') {
        if (depth == 1) {
          object_start = i;
        }
        ++depth;
      } else if (ch == '}') {
        --depth;
        if (depth == 1 && object_start != std::string::npos) {
          const std::string hand = packet.substr(object_start, i - object_start + 1);
          if (hand.find(side) != std::string::npos) {
            return hand;
          }
          object_start = std::string::npos;
        }
      }
    }
    return {};
  }

  static QString scalar_text(const QJsonValue & value)
  {
    if (value.isBool()) {
      return value.toBool() ? "true" : "false";
    }
    if (value.isDouble()) {
      return QString::number(value.toDouble(), 'g', 10);
    }
    if (value.isString()) {
      return value.toString();
    }
    if (value.isNull()) {
      return "null";
    }
    return {};
  }

  static void append_json_flat(QTreeWidget * table, const QString & path, const QJsonValue & value)
  {
    if (value.isObject()) {
      const QJsonObject object = value.toObject();
      for (auto it = object.constBegin(); it != object.constEnd(); ++it) {
        append_json_flat(table, path.isEmpty() ? it.key() : path + "." + it.key(), it.value());
      }
    } else if (value.isArray()) {
      const QJsonArray array = value.toArray();
      for (qsizetype index = 0; index < array.size(); ++index) {
        append_json_flat(table, path + "[" + QString::number(index) + "]", array.at(index));
      }
    } else {
      new QTreeWidgetItem(table, {path, scalar_text(value)});
    }
  }

  static void add_data_section(
    QTreeWidget * table,
    const char * name,
    const char * shape,
    const char * order,
    const char * source)
  {
    auto * row = new QTreeWidgetItem(table, {name, shape, order, source});
    QFont font = row->font(0);
    font.setBold(true);
    for (int column = 0; column < 4; ++column) {
      row->setFont(column, font);
    }
  }

  static QString tuple_text(const QJsonValue & value)
  {
    const QJsonArray tuple = value.toArray();
    QStringList values;
    for (const QJsonValue & component : tuple) {
      values << scalar_text(component);
    }
    return "(" + values.join(", ") + ")";
  }

  static void append_finger_matrix(QTreeWidget * table, const QString & key, const QJsonValue & value)
  {
    const QJsonArray fingers = value.toArray();
    for (qsizetype finger = 0; finger < fingers.size(); ++finger) {
      const QJsonArray joints = fingers.at(finger).toArray();
      QStringList columns;
      columns << key + "[" + QString::number(finger) + "]";
      for (qsizetype joint = 0; joint < 4; ++joint) {
        columns << (joint < joints.size() ? tuple_text(joints.at(joint)) : QString());
      }
      new QTreeWidgetItem(table, columns);
    }
  }

  void add_placeholder(const QString & text)
  {
    data_tree_->clear();
    new QTreeWidgetItem(data_tree_, {"Status", text});
  }

  void show_hand_data(const std::string & hand)
  {
    const QJsonDocument document = QJsonDocument::fromJson(QByteArray::fromStdString(hand));
    if (document.isNull() || !document.isObject()) {
      add_placeholder("The latest hand frame is not valid JSON.");
      return;
    }
    data_tree_->clear();
    const QJsonObject object = document.object();
    for (auto it = object.constBegin(); it != object.constEnd(); ++it) {
      if (it.key() == "hand_angles_rad") {
        add_data_section(data_tree_, "hand_angles_rad", "float[5][3][3]", "finger -> joint -> Euler XYZ; fingers: thumb, index, middle, ring, pinky", "HandLayer::GetHandPose -> HandPose::GetHandAngles; unit rad");
      } else if (it.key() == "joint_positions_mm") {
        add_data_section(data_tree_, "joint_positions_mm", "float[5][4][3]", "finger -> node -> XYZ; unit mm; node order is SDK order", "HandLayer::GetHandPose -> HandPose::GetJointPositions");
      } else if (it.key() == "joint_rotations_xyzw") {
        add_data_section(data_tree_, "joint_rotations_xyzw", "float[5][4][4]", "finger -> node -> [x, y, z, w]; node order is SDK order", "HandLayer::GetHandPose -> HandPose::GetJointRotations");
      }
      if (it.key() == "hand_angles_rad" || it.key() == "joint_positions_mm" || it.key() == "joint_rotations_xyzw") {
        append_finger_matrix(data_tree_, it.key(), it.value());
      } else {
        append_json_flat(data_tree_, it.key(), it.value());
      }
    }
  }

  void start_sending()
  {
    bool valid_port = false;
    const int port = port_->text().toInt(&valid_port);
    if (!valid_port || port <= 0 || port > 65535) {
      QMessageBox::warning(this, "Invalid parameters", "Port must be between 1 and 65535.");
      return;
    }
    bool valid_rate = false;
    const double rate = rate_->text().toDouble(&valid_rate);
    if (!valid_rate || !std::isfinite(rate) || rate <= 0.0) {
      QMessageBox::warning(this, "Invalid parameters", "Send rate must be a positive number in Hz.");
      return;
    }
    const std::string host = host_->text().trimmed().isEmpty()
      ? "127.0.0.1" : host_->text().trimmed().toStdString();
    sockaddr_in address{};
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
      QMessageBox::warning(this, "Invalid parameters", "IP must be an IPv4 address, for example 192.168.1.20.");
      return;
    }
    ReaderOptions options;
    options.host = host;
    options.port = port;
    options.rate_hz = rate;
    options.left = hand_left_->isChecked() || hand_both_->isChecked();
    options.right = hand_right_->isChecked() || hand_both_->isChecked();
    start_sender(sender_, options);
  }

  void start_preset(const char * host, int port)
  {
    host_->setText(QString::fromLatin1(host));
    port_->setText(QString::number(port));
    hand_left_->setChecked(false);
    hand_right_->setChecked(false);
    hand_both_->setChecked(true);
    start_sending();
  }

  void refresh_device_status()
  {
    const ReaderDeviceStatus status = read_device_status();
    sensecom_status_->setText(QString::fromStdString(status.sensecom));
    left_status_->setText(QString::fromStdString(status.left));
    right_status_->setText(QString::fromStdString(status.right));
    left_calibration_status_->setText(QString::fromStdString(status.left_calibration));
    right_calibration_status_->setText(QString::fromStdString(status.right_calibration));
    set_detected(sensecom_status_, status.sensecom_detected);
    set_detected(left_status_, status.left_connected);
    set_detected(right_status_, status.right_connected);
  }

  static void set_detected(QLabel * label, bool detected)
  {
    label->setProperty("detected", detected);
    label->style()->unpolish(label);
    label->style()->polish(label);
  }

  void refresh_packet_preview()
  {
    std::string packet;
    {
      std::lock_guard<std::mutex> lock(sender_.packet_mutex);
      packet = sender_.latest_packet;
    }
    if (!packet.empty()) {
      // The packet timestamp changes on every frame. Compare the complete
      // packet so the visualizer refreshes even when a selected hand happens
      // to hold the same pose for several frames.
      const QString frame = QString::fromStdString(packet);
      if (frame != shown_packet_) {
        const std::string hand = selected_hand_json(packet, data_right_hand_);
        if (hand.empty()) {
          add_placeholder(data_right_hand_ ? "Right hand is not present in the latest frame."
                                            : "Left hand is not present in the latest frame.");
        } else {
          show_hand_data(hand);
        }
        shown_packet_ = frame;
      }
    }
  }

  void refresh()
  {
    refresh_packet_preview();
    if (++refresh_ticks_ % 3 == 0) {
      refresh_device_status();
    }
    const bool sending = sender_.running;
    sending_indicator_->setText(sending
      ? QString::fromLatin1("Sending now  |  packets: ") + QString::number(sender_.sent_packets.load())
      : "Not sending");
    sending_indicator_->setProperty("sending", sending);
    sending_indicator_->style()->unpolish(sending_indicator_);
    sending_indicator_->style()->polish(sending_indicator_);
    start_button_->setEnabled(!sending);
    stop_button_->setEnabled(sending);
  }

  void apply_style()
  {
    setStyleSheet(
      "QWidget { background: #F5F5F7; color: #1D1D1F; font-family: 'Segoe UI'; font-size: 14px; }"
      "QFrame#card { background: #FFFFFF; border: 1px solid #E5E5EA; border-radius: 16px; }"
      "QLabel#windowTitle { font-size: 28px; font-weight: 600; }"
      "QLabel#windowSubtitle, QLabel#cardSubtitle { color: #6E6E73; }"
      "QLabel#cardTitle { font-size: 16px; font-weight: 600; }"
      "QLabel#statusLabel { background: #E5E5EA; color: #6E6E73; border-radius: 8px; padding: 8px 10px; }"
      "QLabel#statusLabel[detected=true] { background: #DFF7E7; color: #1B7A3A; }"
      "QLabel#sendingIndicator { font-weight: 600; color: #6E6E73; }"
      "QLabel#sendingIndicator[sending=true] { color: #34C759; }"
      "QLineEdit, QPlainTextEdit, QTreeWidget { background: #FBFBFD; border: 1px solid #D1D1D6; border-radius: 8px; padding: 7px; }"
      "QLineEdit:focus, QPlainTextEdit:focus, QTreeWidget:focus { border: 1px solid #007AFF; }"
      "QTreeWidget::item { padding: 5px 3px; }"
      "QTreeWidget::item:alternate { background: #F5F5F7; }"
      "QHeaderView::section { background: #F5F5F7; border: 0; border-bottom: 1px solid #D1D1D6; padding: 8px; font-weight: 600; }"
      "QPushButton { background: #E9E9ED; border: 0; border-radius: 9px; padding: 9px 16px; min-width: 110px; }"
      "QPushButton:hover { background: #DCDCE1; }"
      "QPushButton#navigationButton { min-width: 0; padding: 8px 12px; color: #6E6E73; }"
      "QPushButton#navigationButton[selected=true] { background: #E5F1FF; color: #007AFF; font-weight: 600; }"
      "QPushButton#handViewButton { min-width: 0; padding: 8px 14px; color: #6E6E73; }"
      "QPushButton#handViewButton[selected=true] { background: #E5F1FF; color: #007AFF; font-weight: 600; }"
      "QPushButton#primaryButton { background: #007AFF; color: white; font-weight: 600; }"
      "QPushButton#primaryButton:hover { background: #0071E3; }"
      "QPushButton:disabled { background: #E5E5EA; color: #8E8E93; }");
  }

  UdpSender sender_;
  QStackedWidget * pages_{};
  QPushButton * main_tab_{};
  QPushButton * data_tab_{};
  QPushButton * format_tab_{};
  QLineEdit * host_{};
  QLineEdit * port_{};
  QLineEdit * rate_{};
  QRadioButton * hand_left_{};
  QRadioButton * hand_right_{};
  QRadioButton * hand_both_{};
  QButtonGroup * hand_group_{};
  QLabel * sensecom_status_{};
  QLabel * left_status_{};
  QLabel * right_status_{};
  QLabel * left_calibration_status_{};
  QLabel * right_calibration_status_{};
  QLabel * sending_indicator_{};
  QPushButton * start_button_{};
  QPushButton * stop_button_{};
  QPushButton * data_left_button_{};
  QPushButton * data_right_button_{};
  QTreeWidget * data_tree_{};
  QTimer * timer_{};
  QString shown_packet_;
  unsigned int refresh_ticks_{};
  bool data_right_hand_{false};
};

}  // namespace

int run_reader_window(int argc, char ** argv)
{
  QApplication application(argc, argv);
  ReaderWindow window;
  window.show();
  return application.exec();
}
