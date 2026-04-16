#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageEnhancer : public rclcpp::Node {
public:
    ImageEnhancer() : Node("hsr_image_enhancer") {
        // Tópico estándar de la cámara del HSR
        std::string input_topic = "/head_rgbd_sensor/rgb/image_rect_color";
        
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 
            rclcpp::SensorDataQoS(), // QoS optimizado para video (Best Effort)
            std::bind(&ImageEnhancer::image_callback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/hardware/camera/image_raw", 10);
        
        RCLCPP_INFO(this->get_logger(), "HSR Enhancer listo. Escuchando en: %s", input_topic.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Conversión directa para evitar copias innecesarias
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat frame = cv_ptr->image.clone(); // Clonamos solo para modificar
            cv::Mat working_mat;

            // 1. Balance de Blancos (Gray World Rápido)
            cv::Scalar avg = cv::mean(frame);
            double gray_val = (avg[0] + avg[1] + avg[2]) / 3.0;
            if (gray_val > 10.0) {
                // Escalamiento por canal (B G R)
                frame.convertTo(frame, -1, 1.0, 0); // Placeholder si no quieres separar canales
                std::vector<cv::Mat> channels;
                cv::split(frame, channels);
                channels[0] *= (gray_val / avg[0]); // Blue
                channels[1] *= (gray_val / avg[1]); // Green
                channels[2] *= (gray_val / avg[2]); // Red
                cv::merge(channels, frame);
            }

            // 2. Mejora de Contraste (CLAHE) en Espacio Lab
            cv::cvtColor(frame, working_mat, cv::COLOR_BGR2Lab);
            std::vector<cv::Mat> lab_planes(3);
            cv::split(working_mat, lab_planes);
            
            static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
            clahe->apply(lab_planes[0], lab_planes[0]);
            
            cv::merge(lab_planes, working_mat);
            cv::cvtColor(working_mat, frame, cv::COLOR_Lab2BGR);

            // 3. Filtro Bilateral (Suavizado preservando bordes)
            cv::bilateralFilter(frame, working_mat, 5, 50, 50);

            // 4. Unsharp Masking (Enfoque)
            cv::Mat blurred;
            cv::GaussianBlur(working_mat, blurred, cv::Size(0, 0), 2.0);
            cv::addWeighted(working_mat, 1.5, blurred, -0.5, 0, working_mat);

            // 5. Saturación (Para que el HSR detecte mejor objetos de colores)
            cv::cvtColor(working_mat, frame, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> hsv_channels;
            cv::split(frame, hsv_channels);
            hsv_channels[1].convertTo(hsv_channels[1], -1, 1.3, 0); 
            cv::merge(hsv_channels, frame);
            cv::cvtColor(frame, working_mat, cv::COLOR_HSV2BGR);

            // Publicar
            auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", working_mat).toImageMsg();
            image_pub_->publish(*output_msg);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error en HSR Vision: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageEnhancer>());
    rclcpp::shutdown();
    return 0;
}