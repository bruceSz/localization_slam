#pragma once


namespace slam_fe {
class Config {
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}

    ~Config() {}

    static void setParameterFile(const std::string& name);

    template<typename T>
    static T get(const std::string& key) {
        return T(Config::config_->file_[key]);
    }
};
}