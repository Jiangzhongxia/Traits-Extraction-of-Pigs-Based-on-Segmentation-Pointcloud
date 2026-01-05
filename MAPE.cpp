//#include <iostream>
//#include <vector>
//#include <cmath>
//
//// 计算两个数字之间的绝对百分比误差
//double calculateMAPE(const std::vector<double>& actual, const std::vector<double>& predicted) {
//    double sumErrors = 0.0;
//    int count = 0;
//
//    // 确保实际值和预测值的向量长度相同
//    if (actual.size() != predicted.size()) {
//        std::cerr << "Error: The size of actual and predicted vectors must be the same." << std::endl;
//        return -1;
//    }
//
//    for (size_t i = 0; i < actual.size(); ++i) {
//        if (actual[i] == 0) {
//            std::cerr << "Error: Actual value at index " << i << " is zero, cannot divide by zero." << std::endl;
//            return -1;
//        }
//        double error = fabs((actual[i] - predicted[i]) / actual[i]);
//        sumErrors += error;
//        count++;
//    }
//
//    return (sumErrors / count) * 100.0;
//}
//
//int main() {
//    std::vector<double> actualValues = { 32,32,32,32,31,32,32,33,30,32,28,32,29,27,32 };
//    std::vector<double> predictedValues = { 31,24,35,31,32,28,29,38,43,38,36,38,32,41,31 };
//
//    double mape = calculateMAPE(actualValues, predictedValues);
//    if (mape != -1) {
//        std::cout << "The Mean Absolute Percentage Error (MAPE) is: " << mape << "%" << std::endl;
//    }
//
//    return 0;
//}