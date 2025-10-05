// Simple Docking Dashboard - Widget-based
#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>

#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <QWidget>

namespace gz::gui::plugins
{
  /// \brief Dashboard showing docking telemetry
  class DockingDashboardSimple : public QWidget
  {
    Q_OBJECT

  public:
    DockingDashboardSimple(QWidget *parent = nullptr) : QWidget(parent)
    {
      // Create main layout
      auto mainLayout = new QVBoxLayout(this);
      
      // Title
      auto title = new QLabel("<h2>Docking Telemetry</h2>");
      title->setAlignment(Qt::AlignCenter);
      mainLayout->addWidget(title);
      
      // Distance group
      auto distGroup = new QGroupBox("Distance");
      auto distLayout = new QVBoxLayout();
      distanceLabel = new QLabel("CoM: -- m");
      portDistLabel = new QLabel("Port: -- m");
      distLayout->addWidget(distanceLabel);
      distLayout->addWidget(portDistLabel);
      distGroup->setLayout(distLayout);
      mainLayout->addWidget(distGroup);
      
      // Alignment group
      auto alignGroup = new QGroupBox("Alignment");
      auto alignLayout = new QVBoxLayout();
      alignmentLabel = new QLabel("Angle: -- °");
      alignLayout->addWidget(alignmentLabel);
      alignGroup->setLayout(alignLayout);
      mainLayout->addWidget(alignGroup);
      
      // Velocity group
      auto velGroup = new QGroupBox("Velocity");
      auto velLayout = new QVBoxLayout();
      velocityLabel = new QLabel("Relative: -- m/s");
      approachLabel = new QLabel("Approach: -- m/s");
      velLayout->addWidget(velocityLabel);
      velLayout->addWidget(approachLabel);
      velGroup->setLayout(velLayout);
      mainLayout->addWidget(velGroup);
      
      // Thrust group
      auto thrustGroup = new QGroupBox("Thrust");
      auto thrustLayout = new QVBoxLayout();
      thrustLabel = new QLabel("Current: -- N");
      fuelLabel = new QLabel("Fuel: 100%");
      thrustLayout->addWidget(thrustLabel);
      thrustLayout->addWidget(fuelLabel);
      thrustGroup->setLayout(thrustLayout);
      mainLayout->addWidget(thrustGroup);
      
      // Status group
      auto statusGroup = new QGroupBox("Mission Status");
      auto statusLayout = new QVBoxLayout();
      timeLabel = new QLabel("Time: 0.0 s");
      modeLabel = new QLabel("Mode: APPROACH");
      statusLayout->addWidget(timeLabel);
      statusLayout->addWidget(modeLabel);
      statusGroup->setLayout(statusLayout);
      mainLayout->addWidget(statusGroup);
      
      // Success criteria group
      auto successGroup = new QGroupBox("Success Criteria");
      auto successLayout = new QVBoxLayout();
      distCheckLabel = new QLabel("Port < 0.5m: ❌");
      alignCheckLabel = new QLabel("Align < 15°: ❌");
      velCheckLabel = new QLabel("Vel < 0.3m/s: ❌");
      successLayout->addWidget(distCheckLabel);
      successLayout->addWidget(alignCheckLabel);
      successLayout->addWidget(velCheckLabel);
      successGroup->setLayout(successLayout);
      mainLayout->addWidget(successGroup);
      
      mainLayout->addStretch();
      
      // Setup transport
      // Subscribe to telemetry topics
      node.Subscribe("/docking/distance", &DockingDashboardSimple::OnDistance, this);
      node.Subscribe("/docking/port_distance", &DockingDashboardSimple::OnPortDistance, this);
      node.Subscribe("/docking/alignment", &DockingDashboardSimple::OnAlignment, this);
      node.Subscribe("/docking/velocity", &DockingDashboardSimple::OnVelocity, this);
      node.Subscribe("/docking/approach", &DockingDashboardSimple::OnApproach, this);
      node.Subscribe("/docking/thrust", &DockingDashboardSimple::OnThrust, this);
      node.Subscribe("/docking/fuel", &DockingDashboardSimple::OnFuel, this);
      node.Subscribe("/docking/time", &DockingDashboardSimple::OnTime, this);
      
      // Timer to update UI
      auto timer = new QTimer(this);
      connect(timer, &QTimer::timeout, this, &DockingDashboardSimple::UpdateDisplay);
      timer->start(100); // 10 Hz
    }

  private slots:
    void UpdateDisplay()
    {
      // Update success criteria
      if (portDistance < 0.5) {
        distCheckLabel->setText("Port < 0.5m: ✅");
        distCheckLabel->setStyleSheet("color: green;");
      } else {
        distCheckLabel->setText("Port < 0.5m: ❌");
        distCheckLabel->setStyleSheet("color: red;");
      }
      
      if (alignment < 15.0) {
        alignCheckLabel->setText("Align < 15°: ✅");
        alignCheckLabel->setStyleSheet("color: green;");
      } else {
        alignCheckLabel->setText("Align < 15°: ❌");
        alignCheckLabel->setStyleSheet("color: red;");
      }
      
      if (velocity < 0.3) {
        velCheckLabel->setText("Vel < 0.3m/s: ✅");
        velCheckLabel->setStyleSheet("color: green;");
      } else {
        velCheckLabel->setText("Vel < 0.3m/s: ❌");
        velCheckLabel->setStyleSheet("color: red;");
      }
    }

  private:
    void OnDistance(const gz::msgs::Double &msg)
    {
      distance = msg.data();
      distanceLabel->setText(QString("CoM: %1 m").arg(distance, 0, 'f', 2));
    }
    
    void OnPortDistance(const gz::msgs::Double &msg)
    {
      portDistance = msg.data();
      portDistLabel->setText(QString("Port: %1 m").arg(portDistance, 0, 'f', 2));
      
      // Color code
      if (portDistance < 5.0) {
        portDistLabel->setStyleSheet("color: green; font-weight: bold;");
      } else if (portDistance < 20.0) {
        portDistLabel->setStyleSheet("color: orange; font-weight: bold;");
      } else {
        portDistLabel->setStyleSheet("color: red;");
      }
    }
    
    void OnAlignment(const gz::msgs::Double &msg)
    {
      alignment = msg.data();
      alignmentLabel->setText(QString("Angle: %1°").arg(alignment, 0, 'f', 1));
      
      if (alignment < 15.0) {
        alignmentLabel->setStyleSheet("color: green; font-weight: bold;");
      } else if (alignment < 45.0) {
        alignmentLabel->setStyleSheet("color: orange; font-weight: bold;");
      } else {
        alignmentLabel->setStyleSheet("color: red;");
      }
    }
    
    void OnVelocity(const gz::msgs::Double &msg)
    {
      velocity = msg.data();
      velocityLabel->setText(QString("Relative: %1 m/s").arg(velocity, 0, 'f', 3));
    }
    
    void OnApproach(const gz::msgs::Double &msg)
    {
      approach = msg.data();
      approachLabel->setText(QString("Approach: %1 m/s").arg(approach, 0, 'f', 3));
    }
    
    void OnThrust(const gz::msgs::Double &msg)
    {
      thrust = msg.data();
      thrustLabel->setText(QString("Current: %1 N").arg(thrust, 0, 'f', 2));
    }
    
    void OnFuel(const gz::msgs::Double &msg)
    {
      fuel = msg.data();
      fuelLabel->setText(QString("Fuel: %1%").arg(fuel, 0, 'f', 1));
      
      if (fuel > 50.0) {
        fuelLabel->setStyleSheet("color: green;");
      } else if (fuel > 20.0) {
        fuelLabel->setStyleSheet("color: orange;");
      } else {
        fuelLabel->setStyleSheet("color: red;");
      }
    }
    
    void OnTime(const gz::msgs::Double &msg)
    {
      time = msg.data();
      timeLabel->setText(QString("Time: %1 s").arg(time, 0, 'f', 1));
    }

  private:
    gz::transport::Node node;
    
    QLabel *distanceLabel;
    QLabel *portDistLabel;
    QLabel *alignmentLabel;
    QLabel *velocityLabel;
    QLabel *approachLabel;
    QLabel *thrustLabel;
    QLabel *fuelLabel;
    QLabel *timeLabel;
    QLabel *modeLabel;
    
    QLabel *distCheckLabel;
    QLabel *alignCheckLabel;
    QLabel *velCheckLabel;
    
    double distance{0.0};
    double portDistance{0.0};
    double alignment{0.0};
    double velocity{0.0};
    double approach{0.0};
    double thrust{0.0};
    double fuel{100.0};
    double time{0.0};
  };

  /// \brief Plugin wrapper
  class DockingDashboardPlugin : public Plugin
  {
    Q_OBJECT

  public:
    DockingDashboardPlugin()
    {
      gzmsg << "DockingDashboard plugin loading..." << std::endl;
    }

    void LoadConfig(const tinyxml2::XMLElement *) override
    {
      if (!this->title.empty()) {
        auto mainWidget = new DockingDashboardSimple();
        gz::gui::App()->findChild<gz::gui::MainWindow *>()->QuickWindow()->setProperty(
          "dashboardWidget", QVariant::fromValue(mainWidget));
        mainWidget->show();
        gzmsg << "DockingDashboard widget created!" << std::endl;
      }
    }
  };
}

#include "docking_dashboard_simple.moc"

GZ_ADD_PLUGIN(gz::gui::plugins::DockingDashboardPlugin,
              gz::gui::Plugin)
