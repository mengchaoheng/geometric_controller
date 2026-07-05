#include "geometric_controller/reference_trajectory.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

constexpr int kOmegaScale = 100;

int omegaToSlider(double value)
{
  return static_cast<int>(std::lround(value * static_cast<double>(kOmegaScale)));
}

double sliderToOmega(int value)
{
  return static_cast<double>(value) / static_cast<double>(kOmegaScale);
}

}  // namespace

class TrajectoryControlPanel
{
public:
  TrajectoryControlPanel()
  : node_(std::make_shared<rclcpp::Node>("trajectory_control_panel"))
  {
    node_->declare_parameter<std::string>("target_node", "/trajectory_offboard_node");
    target_node_ = node_->get_parameter("target_node").as_string();
    parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, target_node_);

    buildUi();
    configureTimers();
  }

  void show()
  {
    window_->show();
  }

private:
  void buildUi()
  {
    window_ = std::make_unique<QWidget>();
    window_->setWindowTitle("geometric_controller trajectory panel");
    window_->resize(520, 640);

    auto * main_layout = new QVBoxLayout(window_.get());
    main_layout->setContentsMargins(12, 12, 12, 12);
    main_layout->setSpacing(10);

    auto * reference_group = new QGroupBox("Reference");
    auto * reference_form = new QFormLayout(reference_group);
    reference_form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    addOmegaControl(reference_form);
    addTrajectoryCombo(reference_form);
    main_layout->addWidget(reference_group);

    auto * shape_group = new QGroupBox("Shape");
    auto * shape_layout = new QVBoxLayout(shape_group);
    shape_stack_ = new QStackedWidget();
    shape_layout->addWidget(shape_stack_);
    addShapePages();
    main_layout->addWidget(shape_group, 1);

    auto * yaw_group = new QGroupBox("Yaw");
    auto * yaw_form = new QFormLayout(yaw_group);
    addBool(yaw_form, "trajectory_yaw_lock", "trajectory_yaw_lock", false);
    addDouble(yaw_form, "trajectory_yaw_fixed", "trajectory_yaw_fixed", 0.0, -3.1416, 3.1416, 0.01, 3, " rad");
    main_layout->addWidget(yaw_group);

    auto * start_group = new QGroupBox("Start");
    auto * start_form = new QFormLayout(start_group);
    addDouble(
      start_form, "transition_duration_s", "offboard.start_transition_duration_s", 4.0, 0.5,
      20.0, 0.1, 2, " s");
    addDouble(
      start_form, "position_tolerance", "offboard.takeoff_position_tolerance", 0.25, 0.02,
      2.0, 0.01, 2, " m");
    addDouble(
      start_form, "velocity_tolerance", "offboard.takeoff_velocity_tolerance", 0.5, 0.02,
      3.0, 0.01, 2, " m/s");
    main_layout->addWidget(start_group);

    auto * bottom_layout = new QHBoxLayout();
    auto * refresh_button = new QPushButton("Refresh");
    status_label_ = new QLabel();
    status_label_->setMinimumWidth(280);
    bottom_layout->addWidget(refresh_button);
    bottom_layout->addWidget(status_label_, 1);
    main_layout->addLayout(bottom_layout);

    QObject::connect(refresh_button, &QPushButton::clicked, [this]() {
      syncFromTarget();
    });

    setStatus("Waiting for " + QString::fromStdString(target_node_));
  }

  void addOmegaControl(QFormLayout * form)
  {
    registerParameter("omega_value");

    auto * row = new QWidget();
    auto * layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    omega_slider_ = new QSlider(Qt::Horizontal);
    omega_slider_->setRange(omegaToSlider(0.01), omegaToSlider(4.0));
    omega_slider_->setSingleStep(1);
    omega_slider_->setPageStep(10);
    omega_spin_ = new QDoubleSpinBox();
    omega_spin_->setRange(0.01, 4.0);
    omega_spin_->setSingleStep(0.01);
    omega_spin_->setDecimals(3);
    omega_spin_->setSuffix(" rad/s");
    omega_spin_->setValue(0.5);
    omega_slider_->setValue(omegaToSlider(0.5));

    layout->addWidget(omega_slider_, 1);
    layout->addWidget(omega_spin_);
    form->addRow("omega_value", row);

    QObject::connect(omega_slider_, &QSlider::valueChanged, [this](int raw_value) {
      if (updating_ui_) {
        return;
      }
      const double value = sliderToOmega(raw_value);
      {
        const QSignalBlocker blocker(omega_spin_);
        omega_spin_->setValue(value);
      }
      setDoubleParameter("omega_value", value);
    });

    QObject::connect(
      omega_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double value) {
        if (updating_ui_) {
          return;
        }
        {
          const QSignalBlocker blocker(omega_slider_);
          omega_slider_->setValue(omegaToSlider(value));
        }
        setDoubleParameter("omega_value", value);
      });
  }

  void addTrajectoryCombo(QFormLayout * form)
  {
    registerParameter("trajectory_type");

    trajectory_combo_ = new QComboBox();
    for (const auto & name : geometric_controller::supportedTrajectoryTypes()) {
      trajectory_combo_->addItem(
        QString::fromStdString(name), geometric_controller::trajectoryTypeIdFromName(name));
    }
    form->addRow("trajName", trajectory_combo_);

    QObject::connect(
      trajectory_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        if (index < 0) {
          return;
        }
        const int type = trajectory_combo_->itemData(index).toInt();
        shape_stack_->setCurrentIndex(std::max(0, type - 1));
        if (!updating_ui_) {
          setIntegerParameter("trajectory_type", type);
        }
      });
  }

  void addShapePages()
  {
    auto * figure8_horizontal = addShapePage();
    addDouble(figure8_horizontal, "Ax", "figure8_horizontal_Ax", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(figure8_horizontal, "Ay", "figure8_horizontal_Ay", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(figure8_horizontal, "Hc", "figure8_horizontal_Hc", 3.0, 0.5, 10.0, 0.1, 2, " m");
    addDouble(
      figure8_horizontal, "theta0", "figure8_horizontal_theta0", 0.0, -3.1416, 3.1416,
      0.01, 3, " rad");

    auto * figure8_vertical = addShapePage();
    addDouble(figure8_vertical, "Ay", "figure8_vertical_Ay", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(figure8_vertical, "Az", "figure8_vertical_Az", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(figure8_vertical, "Hc", "figure8_vertical_Hc", 3.0, 0.5, 10.0, 0.1, 2, " m");
    addDouble(
      figure8_vertical, "theta0", "figure8_vertical_theta0", -0.7854, -3.1416, 3.1416,
      0.01, 3, " rad");

    auto * helix_flip = addShapePage();
    addDouble(helix_flip, "Ay", "helix_flip_Ay", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(helix_flip, "Az", "helix_flip_Az", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(helix_flip, "Hc", "helix_flip_Hc", 3.0, 0.5, 10.0, 0.1, 2, " m");
    addDouble(helix_flip, "Vx", "helix_flip_Vx", 0.3, -5.0, 5.0, 0.1, 2, " m/s");
    addDouble(helix_flip, "theta0", "helix_flip_theta0", 0.0, -3.1416, 3.1416, 0.01, 3, " rad");

    auto * helix_flip_y = addShapePage();
    addDouble(helix_flip_y, "Ax", "helix_flip_y_Ax", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(helix_flip_y, "Az", "helix_flip_y_Az", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(helix_flip_y, "Hc", "helix_flip_y_Hc", 3.0, 0.5, 10.0, 0.1, 2, " m");
    addDouble(helix_flip_y, "Vy", "helix_flip_y_Vy", 0.3, -5.0, 5.0, 0.1, 2, " m/s");
    addDouble(
      helix_flip_y, "theta0", "helix_flip_y_theta0", 0.0, -3.1416, 3.1416, 0.01,
      3, " rad");

    auto * flip_loop_sine = addShapePage();
    addDouble(flip_loop_sine, "Ay", "flip_loop_sine_Ay", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(flip_loop_sine, "Az", "flip_loop_sine_Az", 2.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(flip_loop_sine, "Hc", "flip_loop_sine_Hc", 3.0, 0.5, 10.0, 0.1, 2, " m");
    addDouble(flip_loop_sine, "Vx", "flip_loop_sine_Vx", 0.0, -5.0, 5.0, 0.1, 2, " m/s");
    addDouble(
      flip_loop_sine, "theta0", "flip_loop_sine_theta0", 0.0, -3.1416, 3.1416,
      0.01, 3, " rad");

    auto * fast_circle = addShapePage();
    addDouble(fast_circle, "Ax", "fast_circle_Ax", 3.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(fast_circle, "Ay", "fast_circle_Ay", 3.0, 0.1, 10.0, 0.1, 2, " m");
    addDouble(fast_circle, "Hc", "fast_circle_Hc", 3.0, 0.5, 10.0, 0.1, 2, " m");
    addDouble(fast_circle, "theta0", "fast_circle_theta0", 0.0, -3.1416, 3.1416, 0.01, 3, " rad");
  }

  QFormLayout * addShapePage()
  {
    auto * page = new QWidget();
    auto * form = new QFormLayout(page);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    shape_stack_->addWidget(page);
    return form;
  }

  QDoubleSpinBox * addDouble(
    QFormLayout * form, const QString & label, const std::string & parameter_name,
    double default_value, double min, double max, double step, int decimals, const QString & suffix)
  {
    registerParameter(parameter_name);

    auto * spin = new QDoubleSpinBox();
    spin->setRange(min, max);
    spin->setSingleStep(step);
    spin->setDecimals(decimals);
    spin->setSuffix(suffix);
    spin->setValue(default_value);
    double_controls_[parameter_name] = spin;
    form->addRow(label, spin);

    QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this, parameter_name](double value) {
      if (!updating_ui_) {
        setDoubleParameter(parameter_name, value);
      }
    });

    return spin;
  }

  QCheckBox * addBool(
    QFormLayout * form, const QString & label, const std::string & parameter_name, bool default_value)
  {
    registerParameter(parameter_name);

    auto * check_box = new QCheckBox();
    check_box->setChecked(default_value);
    bool_controls_[parameter_name] = check_box;
    form->addRow(label, check_box);

    QObject::connect(check_box, &QCheckBox::toggled, [this, parameter_name](bool value) {
      if (!updating_ui_) {
        setBoolParameter(parameter_name, value);
      }
    });

    return check_box;
  }

  void configureTimers()
  {
    ros_timer_ = new QTimer(window_.get());
    QObject::connect(ros_timer_, &QTimer::timeout, [this]() {
      rclcpp::spin_some(node_);
    });
    ros_timer_->start(20);

    sync_timer_ = new QTimer(window_.get());
    QObject::connect(sync_timer_, &QTimer::timeout, [this]() {
      if (!initial_sync_done_) {
        syncFromTarget();
      }
    });
    sync_timer_->start(500);
  }

  void syncFromTarget()
  {
    if (sync_in_progress_) {
      return;
    }
    if (!parameter_client_->service_is_ready()) {
      setStatus("Waiting for " + QString::fromStdString(target_node_));
      return;
    }

    sync_in_progress_ = true;
    const auto parameter_names = parameter_names_;
    parameter_client_->get_parameters(
      parameter_names,
      [this, parameter_names](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        sync_in_progress_ = false;
        try {
          applyTargetParameters(parameter_names, future.get());
          initial_sync_done_ = true;
          setStatus("Connected to " + QString::fromStdString(target_node_));
        } catch (const std::exception & exception) {
          setStatus("Sync failed: " + QString::fromStdString(exception.what()));
        }
      });
  }

  void applyTargetParameters(
    const std::vector<std::string> & names, const std::vector<rclcpp::Parameter> & parameters)
  {
    updating_ui_ = true;

    for (size_t i = 0; i < names.size() && i < parameters.size(); ++i) {
      const auto & name = names[i];
      const auto & parameter = parameters[i];
      if (name == "trajectory_type" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        setTrajectoryTypeUi(static_cast<int>(parameter.as_int()));
        continue;
      }
      if (name == "omega_value" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        setOmegaUi(parameter.as_double());
        continue;
      }

      const auto double_iter = double_controls_.find(name);
      if (double_iter != double_controls_.end() &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        const QSignalBlocker blocker(double_iter->second);
        double_iter->second->setValue(parameter.as_double());
        continue;
      }

      const auto bool_iter = bool_controls_.find(name);
      if (bool_iter != bool_controls_.end() &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        const QSignalBlocker blocker(bool_iter->second);
        bool_iter->second->setChecked(parameter.as_bool());
      }
    }

    updating_ui_ = false;
  }

  void setTrajectoryTypeUi(int type)
  {
    const int clamped = std::clamp(type, 1, 6);
    const QSignalBlocker blocker(trajectory_combo_);
    const int index = trajectory_combo_->findData(clamped);
    if (index >= 0) {
      trajectory_combo_->setCurrentIndex(index);
    }
    shape_stack_->setCurrentIndex(clamped - 1);
  }

  void setOmegaUi(double value)
  {
    const double clamped = std::clamp(value, 0.01, 4.0);
    const QSignalBlocker spin_blocker(omega_spin_);
    const QSignalBlocker slider_blocker(omega_slider_);
    omega_spin_->setValue(clamped);
    omega_slider_->setValue(omegaToSlider(clamped));
  }

  void setDoubleParameter(const std::string & name, double value)
  {
    setParameter(rclcpp::Parameter(name, value));
  }

  void setIntegerParameter(const std::string & name, int value)
  {
    setParameter(rclcpp::Parameter(name, static_cast<int64_t>(value)));
  }

  void setBoolParameter(const std::string & name, bool value)
  {
    setParameter(rclcpp::Parameter(name, value));
  }

  void setParameter(const rclcpp::Parameter & parameter)
  {
    if (!parameter_client_->service_is_ready()) {
      setStatus("Target not ready");
      return;
    }

    const auto name = parameter.get_name();
    parameter_client_->set_parameters(
      {parameter},
      [this, name](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        try {
          const auto results = future.get();
          if (!results.empty() && !results.front().successful) {
            setStatus(
              "Rejected " + QString::fromStdString(name) + ": " +
              QString::fromStdString(results.front().reason));
            return;
          }
          setStatus("Updated " + QString::fromStdString(name));
        } catch (const std::exception & exception) {
          setStatus("Update failed: " + QString::fromStdString(exception.what()));
        }
      });
  }

  void registerParameter(const std::string & name)
  {
    if (std::find(parameter_names_.begin(), parameter_names_.end(), name) == parameter_names_.end()) {
      parameter_names_.push_back(name);
    }
  }

  void setStatus(const QString & text)
  {
    if (status_label_) {
      status_label_->setText(text);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client_;
  std::string target_node_;
  std::unique_ptr<QWidget> window_;
  QLabel * status_label_{nullptr};
  QComboBox * trajectory_combo_{nullptr};
  QStackedWidget * shape_stack_{nullptr};
  QSlider * omega_slider_{nullptr};
  QDoubleSpinBox * omega_spin_{nullptr};
  QTimer * ros_timer_{nullptr};
  QTimer * sync_timer_{nullptr};
  std::vector<std::string> parameter_names_;
  std::unordered_map<std::string, QDoubleSpinBox *> double_controls_;
  std::unordered_map<std::string, QCheckBox *> bool_controls_;
  bool updating_ui_{false};
  bool initial_sync_done_{false};
  bool sync_in_progress_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  TrajectoryControlPanel panel;
  panel.show();

  const int result = app.exec();
  rclcpp::shutdown();
  return result;
}
