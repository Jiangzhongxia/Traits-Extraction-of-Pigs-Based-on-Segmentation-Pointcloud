#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <map>
#include <functional>
#include <memory>

// 前向声明ONNX Runtime相关类型
struct OrtEnv;
struct OrtSession;

/**
 * @brief ONNX点云分割器类
 * 使用ONNX模型对点云进行分割，识别猪的不同部位
 */
class ONNXSegmenter {
public:
    struct ONNXModelConfig {
        std::string model_path;
        int input_size = 8192; // 默认输入点数
        float confidence_threshold = 0.7f;
        std::vector<std::string> class_names = {
            "background", "head", "torso", "front_left_leg",
            "front_right_leg", "hind_left_leg", "hind_right_leg",
            "tail", "ear", "neck"
        };
    };

    struct SegmentationResult {
        std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> segments;
        std::vector<int> class_labels; // 每个点的类别标签
        float confidence_score = 0.0f; // 整体置信度
    };

    ONNXSegmenter();
    ~ONNXSegmenter();

    /**
     * @brief 加载ONNX模型
     * @param config 模型配置
     * @return 是否加载成功
     */
    bool loadModel(const ONNXModelConfig& config);

    /**
     * @brief 对点云进行分割
     * @param input 输入点云
     * @return 分割结果
     */
    SegmentationResult segment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);

    /**
     * @brief 重新采样点云到固定大小
     * @param input 输入点云
     * @param target_size 目标点数
     * @return 重采样后的点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr resampleCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        int target_size);

    /**
     * @brief 获取可用的类别名称
     * @return 类别名称列表
     */
    std::vector<std::string> getAvailableClasses() const;

    /**
     * @brief 检查模型是否已加载
     * @return 模型是否已加载
     */
    bool isModelLoaded() const;

    /**
     * @brief 设置处理进度回调函数
     * @param callback 回调函数
     */
    void setProgressCallback(std::function<void(double progress, const std::string& stage)> callback);

    /**
     * @brief 设置统计信息回调函数
     * @param callback 回调函数
     */
    void setStatsCallback(std::function<void(const std::string& stage, const std::string& stats)> callback);

    /**
     * @brief 保存分割结果到文件
     * @param result 分割结果
     * @param base_filename 基础文件名
     * @param format 输出格式 ("ply", "xyz")
     */
    void saveSegmentationResult(const SegmentationResult& result,
                               const std::string& base_filename,
                               const std::string& format = "ply");

private:
    ONNXModelConfig config_;
    bool model_loaded_;
    OrtEnv* ort_env_;
    OrtSession* ort_session_;
    std::function<void(double progress, const std::string& stage)> progress_callback_;
    std::function<void(const std::string& stage, const std::string& stats)> stats_callback_;

    /**
     * @brief 初始化ONNX Runtime环境
     * @return 是否初始化成功
     */
    bool initializeONNXRuntime();

    /**
     * @brief 释放ONNX Runtime资源
     */
    void releaseONNXRuntime();

    /**
     * @brief 运行ONNX模型推理
     * @param input_points 输入点数据 (size: N x 3)
     * @param num_points 点的数量
     * @return 预测的类别标签
     */
    std::vector<int> runInference(const float* input_points, int num_points);

    /**
     * @brief 应用统计回调
     * @param stage 阶段名称
     * @param stats 统计信息
     */
    void callStatsCallback(const std::string& stage, const std::string& stats) const;

    /**
     * @brief 应用进度回调
     * @param progress 进度(0-1)
     * @param stage 阶段名称
     */
    void callProgressCallback(double progress, const std::string& stage) const;
};