// docking_dashboard.cpp
// GUI plugin for Gazebo that displays real-time docking telemetry

#include <gz/gui/Plugin.hh>
#include <gz/gui/qt.h>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <QPalette>
#include <QFont>

namespace gz::gui::plugins
{
  class DockingDashboard : public Plugin
  {
    Q_OBJECT

  public:
    DockingDashboard() : Plugin()
    {
      // Create UI
      auto mainLayout = new QVBoxLayout();
      
      // Title
      auto titleLabel = new QLabel("🛰️ ORBITAL DOCKING DASHBOARD");
      QFont titleFont = titleLabel->font();
      titleFont.setPointSize(16);
      titleFont.setBold(true);
      titleLabel->setFont(titleFont);
      titleLabel->setAlignment(Qt::AlignCenter);
      titleLabel->setStyleSheet("color: #4CAF50; padding: 10px; background-color: #1a1a1a;");
      mainLayout->addWidget(titleLabel);
      
      // === DISTANCE GROUP ===
      auto distanceGroup = new QGroupBox("📏 DISTANCE");
      auto distanceLayout = new QVBoxLayout();
      
      distanceLabel = new QLabel("Distance: -- m");
      portDistanceLabel = new QLabel("Port Distance: -- m");
      distanceLabel->setStyleSheet("font-size: 14pt; color: #2196F3;");
      portDistanceLabel->setStyleSheet("font-size: 14pt; color: #FF9800;");
      
      distanceLayout->addWidget(distanceLabel);
      distanceLayout->addWidget(portDistanceLabel);
      distanceGroup->setLayout(distanceLayout);
      mainLayout->addWidget(distanceGroup);
      
      // === ALIGNMENT GROUP ===
      auto alignmentGroup = new QGroupBox("🎯 ALIGNMENT");
      auto alignmentLayout = new QVBoxLayout();
      
      alignmentLabel = new QLabel("Angle: -- °");
      alignmentStatusLabel = new QLabel("Status: --");
      alignmentLabel->setStyleSheet("font-size: 14pt; color: #9C27B0;");
      alignmentStatusLabel->setStyleSheet("font-size: 12pt;");
      
      alignmentLayout->addWidget(alignmentLabel);
      alignmentLayout->addWidget(alignmentStatusLabel);
      alignmentGroup->setLayout(alignmentLayout);
      mainLayout->addWidget(alignmentGroup);
      
      // === VELOCITY GROUP ===
      auto velocityGroup = new QGroupBox("🚀 VELOCITY");
      auto velocityLayout = new QVBoxLayout();
      
      velocityLabel = new QLabel("Relative Vel: -- m/s");
      approachLabel = new QLabel("Approach Rate: -- m/s");
      velocityLabel->setStyleSheet("font-size: 14pt; color: #F44336;");
      approachLabel->setStyleSheet("font-size: 12pt; color: #FF5722;");
      
      velocityLayout->addWidget(velocityLabel);
      velocityLayout->addWidget(approachLabel);
      velocityGroup->setLayout(velocityLayout);
      mainLayout->addWidget(velocityGroup);
      
      // === THRUST GROUP ===
      auto thrustGroup = new QGroupBox("🔥 THRUST");
      auto thrustLayout = new QVBoxLayout();
      
      thrustLabel = new QLabel("Force: -- N");
      fuelLabel = new QLabel("Fuel Used: -- kg");
      thrustLabel->setStyleSheet("font-size: 14pt; color: #FF9800;");
      fuelLabel->setStyleSheet("font-size: 12pt; color: #FFC107;");
      
      thrustLayout->addWidget(thrustLabel);
      thrustLayout->addWidget(fuelLabel);
      thrustGroup->setLayout(thrustLayout);
      mainLayout->addWidget(thrustGroup);
      
      // === STATUS GROUP ===
      auto statusGroup = new QGroupBox("📊 MISSION STATUS");
      auto statusLayout = new QVBoxLayout();
      
      missionTimeLabel = new QLabel("Time: 0.0 s");
      modeLabel = new QLabel("Mode: INITIALIZING");
      successLabel = new QLabel("Docking: NOT READY");
      
      missionTimeLabel->setStyleSheet("font-size: 12pt;");
      modeLabel->setStyleSheet("font-size: 14pt; font-weight: bold; color: #03A9F4;");
      successLabel->setStyleSheet("font-size: 14pt; font-weight: bold; color: #FF5722;");
      
      statusLayout->addWidget(missionTimeLabel);
      statusLayout->addWidget(modeLabel);
      statusLayout->addWidget(successLabel);
      statusGroup->setLayout(statusLayout);
      mainLayout->addWidget(statusGroup);
      
      // === SUCCESS CRITERIA ===
      auto criteriaGroup = new QGroupBox("✅ SUCCESS CRITERIA");
      auto criteriaLayout = new QVBoxLayout();
      
      distanceCriteria = new QLabel("❌ Distance < 0.5 m");
      velocityCriteria = new QLabel("❌ Velocity < 0.3 m/s");
      alignmentCriteria = new QLabel("❌ Alignment < 15°");
      
      distanceCriteria->setStyleSheet("font-size: 11pt;");
      velocityCriteria->setStyleSheet("font-size: 11pt;");
      alignmentCriteria->setStyleSheet("font-size: 11pt;");
      
      criteriaLayout->addWidget(distanceCriteria);
      criteriaLayout->addWidget(velocityCriteria);
      criteriaLayout->addWidget(alignmentCriteria);
      criteriaGroup->setLayout(criteriaLayout);
      mainLayout->addWidget(criteriaGroup);
      
      mainLayout->addStretch();
      
      // Set main widget
      auto mainWidget = new QWidget();
      mainWidget->setLayout(mainLayout);
      mainWidget->setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #555; border-radius: 5px; margin-top: 10px; padding: 10px; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }");
      
      this->setLayout(new QVBoxLayout());
      this->layout()->addWidget(mainWidget);
      
      // Subscribe to topics (published from controller)
      node.Subscribe("/docking/distance", &DockingDashboard::OnDistance, this);
      node.Subscribe("/docking/port_distance", &DockingDashboard::OnPortDistance, this);
      node.Subscribe("/docking/alignment", &DockingDashboard::OnAlignment, this);
      node.Subscribe("/docking/velocity", &DockingDashboard::OnVelocity, this);
      node.Subscribe("/docking/approach", &DockingDashboard::OnApproach, this);
      node.Subscribe("/docking/thrust", &DockingDashboard::OnThrust, this);
      node.Subscribe("/docking/fuel", &DockingDashboard::OnFuel, this);
      node.Subscribe("/docking/time", &DockingDashboard::OnTime, this);
      node.Subscribe("/docking/mode", &DockingDashboard::OnMode, this);
      
      // Update timer for animations
      auto timer = new QTimer(this);
      connect(timer, &QTimer::timeout, this, &DockingDashboard::Update);
      timer->start(100); // 10Hz update
    }

  private slots:
    void Update()
    {
      // Update success criteria indicators
      if (portDist > 0 && portDist < 0.5)
        distanceCriteria->setText("✅ Distance < 0.5 m");
      else
        distanceCriteria->setText("❌ Distance < 0.5 m");
      
      if (relVel > 0 && relVel < 0.3)
        velocityCriteria->setText("✅ Velocity < 0.3 m/s");
      else
        velocityCriteria->setText("❌ Velocity < 0.3 m/s");
      
      if (alignAngle >= 0 && alignAngle < 15.0)
        alignmentCriteria->setText("✅ Alignment < 15°");
      else
        alignmentCriteria->setText("❌ Alignment < 15°");
      
      // Check overall success
      if (portDist > 0 && portDist < 0.5 && relVel < 0.3 && alignAngle < 15.0)
      {
        successLabel->setText("Docking: ✅ READY");
        successLabel->setStyleSheet("font-size: 14pt; font-weight: bold; color: #4CAF50;");
      }
      else if (portDist < 2.0)
      {
        successLabel->setText("Docking: 🟡 APPROACHING");
        successLabel->setStyleSheet("font-size: 14pt; font-weight: bold; color: #FFC107;");
      }
      else
      {
        successLabel->setText("Docking: 🔴 FAR");
        successLabel->setStyleSheet("font-size: 14pt; font-weight: bold; color: #FF5722;");
      }
      
      // Alignment status
      if (alignAngle < 15.0)
        alignmentStatusLabel->setText("Status: ✅ ALIGNED");
      else if (alignAngle < 45.0)
        alignmentStatusLabel->setText("Status: 🟡 PARTIAL");
      else
        alignmentStatusLabel->setText("Status: 🔴 MISALIGNED");
    }

  private:
    void OnDistance(const gz::msgs::Double &msg)
    {
      dist = msg.data();
      distanceLabel->setText(QString("Distance: %1 m").arg(dist, 0, 'f', 2));
    }
    
    void OnPortDistance(const gz::msgs::Double &msg)
    {
      portDist = msg.data();
      portDistanceLabel->setText(QString("Port Distance: %1 m").arg(portDist, 0, 'f', 2));
    }
    
    void OnAlignment(const gz::msgs::Double &msg)
    {
      alignAngle = msg.data();
      alignmentLabel->setText(QString("Angle: %1°").arg(alignAngle, 0, 'f', 1));
    }
    
    void OnVelocity(const gz::msgs::Double &msg)
    {
      relVel = msg.data();
      velocityLabel->setText(QString("Relative Vel: %1 m/s").arg(relVel, 0, 'f', 3));
    }
    
    void OnApproach(const gz::msgs::Double &msg)
    {
      approach = msg.data();
      approachLabel->setText(QString("Approach Rate: %1 m/s").arg(approach, 0, 'f', 3));
    }
    
    void OnThrust(const gz::msgs::Double &msg)
    {
      thrust = msg.data();
      thrustLabel->setText(QString("Force: %1 N").arg(thrust, 0, 'f', 2));
    }
    
    void OnFuel(const gz::msgs::Double &msg)
    {
      fuel = msg.data();
      fuelLabel->setText(QString("Fuel Used: %1 kg").arg(fuel, 0, 'f', 3));
    }
    
    void OnTime(const gz::msgs::Double &msg)
    {
      missionTime = msg.data();
      missionTimeLabel->setText(QString("Time: %1 s").arg(missionTime, 0, 'f', 1));
    }
    
    void OnMode(const gz::msgs::Double &msg)
    {
      int mode = static_cast<int>(msg.data());
      QString modeText;
      QString modeColor;
      
      switch(mode)
      {
        case 0: modeText = "Mode: CRUISE"; modeColor = "#03A9F4"; break;
        case 1: modeText = "Mode: BRAKE"; modeColor = "#FF9800"; break;
        case 2: modeText = "Mode: FINAL APPROACH"; modeColor = "#FFC107"; break;
        case 3: modeText = "Mode: DOCKING"; modeColor = "#4CAF50"; break;
        default: modeText = "Mode: UNKNOWN"; modeColor = "#888"; break;
      }
      
      modeLabel->setText(modeText);
      modeLabel->setStyleSheet(QString("font-size: 14pt; font-weight: bold; color: %1;").arg(modeColor));
    }

    // UI elements
    QLabel *distanceLabel;
    QLabel *portDistanceLabel;
    QLabel *alignmentLabel;
    QLabel *alignmentStatusLabel;
    QLabel *velocityLabel;
    QLabel *approachLabel;
    QLabel *thrustLabel;
    QLabel *fuelLabel;
    QLabel *missionTimeLabel;
    QLabel *modeLabel;
    QLabel *successLabel;
    QLabel *distanceCriteria;
    QLabel *velocityCriteria;
    QLabel *alignmentCriteria;
    
    // Data
    double dist = 0.0;
    double portDist = 0.0;
    double alignAngle = 0.0;
    double relVel = 0.0;
    double approach = 0.0;
    double thrust = 0.0;
    double fuel = 0.0;
    double missionTime = 0.0;
    
    gz::transport::Node node;
  };
}

#include "docking_dashboard.moc"

GZ_ADD_PLUGIN(gz::gui::plugins::DockingDashboard,
              gz::gui::Plugin)
