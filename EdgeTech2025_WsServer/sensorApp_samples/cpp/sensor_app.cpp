/**
 * C++ センサーデータ取得アプリ
 * Raspberry Pi WebSocketサーバーと連携してセンサーデータを送信
 */

#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <random>

// JSON処理
#include <json/json.h>

// WebSocket通信（簡易実装）
#include <boost/asio.hpp>
#include <boost/thread.hpp>

// OpenCV（利用可能な場合）
#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

// WiringPi（Raspberry Pi用）
#ifdef USE_WIRINGPI
#include <wiringPi.h>
#include <wiringPiI2C.h>
#endif

using namespace std;
using namespace boost::asio;

class SensorApp {
private:
    // WebSocket接続情報
    string server_url_;
    string server_host_;
    string server_port_;
    
    // 実行制御
    atomic<bool> running_;
    atomic<bool> connected_;
    atomic<bool> registered_;
    
    // スレッド
    unique_ptr<thread> data_thread_;
    unique_ptr<thread> connection_thread_;
    
    // センサーデータ
    struct SensorData {
        string worker_status = "waiting";
        string robot_state = "waiting";
        string robot_grip = "closed";
        int screw_count = 0;
        int bolt_count = 0;
        string work_step = "waiting";
        string image_data = "";
        string timestamp = "";
    } sensor_data_;
    
    mutable mutex data_mutex_;
    
    // ログ出力
    void log(const string& level, const string& message) {
        auto now = chrono::system_clock::now();
        auto time_t = chrono::system_clock::to_time_t(now);
        auto tm = *localtime(&time_t);
        
        cout << "[" << put_time(&tm, "%Y-%m-%d %H:%M:%S") << "] "
             << level << ": " << message << endl;
    }
    
    // WebSocket接続（簡易実装）
    void connect_websocket() {
        log("INFO", "Connecting to WebSocket server: " + server_url_);
        
        try {
            // 実際の実装では、websocketppやsimilar libraryを使用
            // ここでは簡易的なHTTP接続でシミュレート
            
            connected_ = true;
            log("INFO", "✅ Connected to WebSocket server");
            
            // クライアント登録
            register_client();
            
            // 接続維持ループ
            while (running_ && connected_) {
                this_thread::sleep_for(chrono::seconds(1));
                
                // ハートビート処理
                if (registered_) {
                    // ping/pong処理
                }
            }
            
        } catch (const exception& e) {
            log("ERROR", "Connection error: " + string(e.what()));
            connected_ = false;
        }
    }
    
    void register_client() {
        log("INFO", "📝 Registering as sensor client...");
        
        // 登録メッセージ作成
        Json::Value register_msg;
        register_msg["client_type"] = "sensor";
        
        // 送信（実際の実装ではWebSocket経由）
        send_message("register_client", register_msg);
        
        // 登録完了をシミュレート
        this_thread::sleep_for(chrono::milliseconds(100));
        registered_ = true;
        
        log("INFO", "✅ Registered successfully");
    }
    
    void send_message(const string& event, const Json::Value& data) {
        // 実際の実装では、WebSocketライブラリを使用してメッセージ送信
        Json::StreamWriterBuilder builder;
        string json_string = Json::writeString(builder, data);
        
        log("DEBUG", "Sending " + event + ": " + json_string);
        
        // ここで実際のWebSocket送信処理
    }
    
    // センサーデータ収集
    void collect_sensor_data() {
        lock_guard<mutex> lock(data_mutex_);
        
        // 作業者状態検出
        sensor_data_.worker_status = detect_worker_status();
        
        // ロボット状態取得
        auto robot_status = get_robot_status();
        sensor_data_.robot_state = robot_status.first;
        sensor_data_.robot_grip = robot_status.second;
        
        // カウント更新
        update_counts();
        
        // 作業ステップ
        sensor_data_.work_step = sensor_data_.worker_status;
        
        // タイムスタンプ
        auto now = chrono::system_clock::now();
        auto time_t = chrono::system_clock::to_time_t(now);
        auto tm = *gmtime(&time_t);
        
        stringstream ss;
        ss << put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
        sensor_data_.timestamp = ss.str();
        
        // 画像データ（オプション）
        #ifdef USE_OPENCV
        sensor_data_.image_data = capture_image();
        #endif
    }
    
    string detect_worker_status() {
        // 実際の実装では、カメラ画像解析やセンサーデータから判定
        static random_device rd;
        static mt19937 gen(rd());
        static uniform_real_distribution<> dis(0.0, 1.0);
        
        vector<string> states = {"waiting", "screw_tightening", "bolt_tightening", "tool_handover", "absent"};
        
        // 80%の確率で前回と同じ状態を維持
        static string last_status = "waiting";
        if (dis(gen) > 0.2) {
            return last_status;
        }
        
        // 20%の確率で状態変更
        uniform_int_distribution<> state_dis(0, states.size() - 1);
        last_status = states[state_dis(gen)];
        
        if (last_status != "waiting") {
            log("INFO", "👷 Worker status changed to: " + last_status);
        }
        
        return last_status;
    }
    
    pair<string, string> get_robot_status() {
        // 実際の実装では、ロボットAPIから状態を取得
        static random_device rd;
        static mt19937 gen(rd());
        static uniform_real_distribution<> dis(0.0, 1.0);
        
        static string last_state = "waiting";
        static string last_grip = "closed";
        
        // 90%の確率で前回と同じ状態を維持
        if (dis(gen) > 0.1) {
            return make_pair(last_state, last_grip);
        }
        
        // 10%の確率で状態変更
        vector<string> states = {"waiting", "operating"};
        vector<string> grips = {"open", "closed"};
        
        uniform_int_distribution<> state_dis(0, states.size() - 1);
        uniform_int_distribution<> grip_dis(0, grips.size() - 1);
        
        last_state = states[state_dis(gen)];
        last_grip = grips[grip_dis(gen)];
        
        log("INFO", "🤖 Robot status changed: " + last_state + ", grip: " + last_grip);
        
        return make_pair(last_state, last_grip);
    }
    
    void update_counts() {
        static random_device rd;
        static mt19937 gen(rd());
        static uniform_real_distribution<> dis(0.0, 1.0);
        
        // ネジ締め作業中の場合、5%の確率でカウント増加
        if (sensor_data_.worker_status == "screw_tightening" && dis(gen) > 0.95) {
            sensor_data_.screw_count++;
            log("INFO", "🔩 Screw count increased: " + to_string(sensor_data_.screw_count));
        }
        
        // ボルト締め作業中の場合、3%の確率でカウント増加
        if (sensor_data_.worker_status == "bolt_tightening" && dis(gen) > 0.97) {
            sensor_data_.bolt_count++;
            log("INFO", "🔧 Bolt count increased: " + to_string(sensor_data_.bolt_count));
        }
    }
    
    #ifdef USE_OPENCV
    string capture_image() {
        static cv::VideoCapture cap(0);  // カメラ0を使用
        
        if (!cap.isOpened()) {
            return "";
        }
        
        cv::Mat frame;
        cap >> frame;
        
        if (frame.empty()) {
            return "";
        }
        
        // JPEG圧縮
        vector<uchar> buffer;
        vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", frame, buffer, compression_params);
        
        // Base64エンコード（簡易実装）
        // 実際の実装では、適切なBase64ライブラリを使用
        return "data:image/jpeg;base64,<base64_encoded_data>";
    }
    #endif
    
    // GPIO制御（Raspberry Pi用）
    #ifdef USE_WIRINGPI
    void setup_gpio() {
        if (wiringPiSetup() == -1) {
            log("ERROR", "WiringPi setup failed");
            return;
        }
        
        // 振動センサー用ピン設定
        pinMode(0, INPUT);  // GPIO 17 (WiringPi pin 0)
        pullUpDnControl(0, PUD_DOWN);
        
        log("INFO", "GPIO setup completed");
    }
    
    bool read_vibration_sensor() {
        return digitalRead(0) == HIGH;
    }
    
    void setup_i2c() {
        // I2Cデバイス初期化
        int fd = wiringPiI2CSetup(0x48);  // アドレス0x48のデバイス
        if (fd == -1) {
            log("ERROR", "I2C setup failed");
            return;
        }
        
        log("INFO", "I2C setup completed");
    }
    
    float read_torque_sensor() {
        // I2Cからトルク値読み取り
        // 実際の実装では、センサー固有のプロトコルを使用
        static random_device rd;
        static mt19937 gen(rd());
        static uniform_real_distribution<> dis(0.0, 100.0);
        
        return dis(gen);  // モック値
    }
    #endif
    
    // データ送信ループ
    void data_sender_loop() {
        log("INFO", "🚀 Starting sensor data transmission...");
        
        while (running_ && connected_ && registered_) {
            try {
                // センサーデータ収集
                collect_sensor_data();
                
                // JSONメッセージ作成
                Json::Value sensor_msg;
                {
                    lock_guard<mutex> lock(data_mutex_);
                    sensor_msg["worker_status"] = sensor_data_.worker_status;
                    sensor_msg["robot_status"]["state"] = sensor_data_.robot_state;
                    sensor_msg["robot_status"]["grip"] = sensor_data_.robot_grip;
                    sensor_msg["screw_count"] = sensor_data_.screw_count;
                    sensor_msg["bolt_count"] = sensor_data_.bolt_count;
                    sensor_msg["work_step"] = sensor_data_.work_step;
                    sensor_msg["timestamp"] = sensor_data_.timestamp;
                    
                    if (!sensor_data_.image_data.empty()) {
                        sensor_msg["image"] = sensor_data_.image_data;
                    }
                }
                
                // WebSocketサーバーに送信
                send_message("sensor_data", sensor_msg);
                
                log("INFO", "📤 Sensor data sent: worker=" + sensor_data_.worker_status + 
                           ", screws=" + to_string(sensor_data_.screw_count) + 
                           ", bolts=" + to_string(sensor_data_.bolt_count));
                
                // 1秒間隔
                this_thread::sleep_for(chrono::seconds(1));
                
            } catch (const exception& e) {
                log("ERROR", "Data sender error: " + string(e.what()));
                this_thread::sleep_for(chrono::seconds(1));
            }
        }
        
        log("INFO", "⏹️ Stopped sensor data transmission");
    }
    
    void parse_server_url(const string& url) {
        // 簡易URL解析
        // ws://hostname:port 形式を想定
        size_t protocol_end = url.find("://");
        if (protocol_end == string::npos) {
            throw invalid_argument("Invalid URL format");
        }
        
        string host_port = url.substr(protocol_end + 3);
        size_t colon_pos = host_port.find(':');
        
        if (colon_pos != string::npos) {
            server_host_ = host_port.substr(0, colon_pos);
            server_port_ = host_port.substr(colon_pos + 1);
        } else {
            server_host_ = host_port;
            server_port_ = "3001";  // デフォルトポート
        }
    }

public:
    SensorApp(const string& server_url = "ws://localhost:3001") 
        : server_url_(server_url), running_(false), connected_(false), registered_(false) {
        
        parse_server_url(server_url);
        log("INFO", "Initializing sensor app for server: " + server_url_);
        
        #ifdef USE_WIRINGPI
        setup_gpio();
        setup_i2c();
        #endif
    }
    
    ~SensorApp() {
        stop();
    }
    
    void start() {
        if (running_) {
            return;
        }
        
        running_ = true;
        log("INFO", "Starting sensor application...");
        
        // 接続スレッド開始
        connection_thread_ = make_unique<thread>(&SensorApp::connect_websocket, this);
        
        // 登録完了まで待機
        while (running_ && (!connected_ || !registered_)) {
            this_thread::sleep_for(chrono::milliseconds(100));
        }
        
        if (running_ && connected_ && registered_) {
            // データ送信スレッド開始
            data_thread_ = make_unique<thread>(&SensorApp::data_sender_loop, this);
        }
    }
    
    void stop() {
        if (!running_) {
            return;
        }
        
        log("INFO", "🛑 Stopping sensor application...");
        running_ = false;
        connected_ = false;
        registered_ = false;
        
        // スレッド終了待機
        if (data_thread_ && data_thread_->joinable()) {
            data_thread_->join();
        }
        
        if (connection_thread_ && connection_thread_->joinable()) {
            connection_thread_->join();
        }
        
        log("INFO", "Sensor application stopped");
    }
    
    void wait() {
        if (data_thread_ && data_thread_->joinable()) {
            data_thread_->join();
        }
        
        if (connection_thread_ && connection_thread_->joinable()) {
            connection_thread_->join();
        }
    }
    
    bool is_running() const {
        return running_;
    }
    
    bool is_connected() const {
        return connected_;
    }
};

// グローバル変数（シグナルハンドラー用）
unique_ptr<SensorApp> g_sensor_app;

// シグナルハンドラー
void signal_handler(int signum) {
    cout << "\n🛑 Received signal " << signum << ", shutting down gracefully..." << endl;
    
    if (g_sensor_app) {
        g_sensor_app->stop();
    }
    
    exit(0);
}

int main(int argc, char* argv[]) {
    // 環境変数からサーバーURL取得
    const char* env_url = getenv("WEBSOCKET_URL");
    string server_url = env_url ? env_url : "ws://localhost:3001";
    
    // コマンドライン引数処理
    if (argc > 1) {
        server_url = argv[1];
    }
    
    cout << "🚀 Starting C++ Sensor App" << endl;
    cout << "📡 Target server: " << server_url << endl;
    
    try {
        // シグナルハンドラー設定
        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
        
        // センサーアプリ作成・開始
        g_sensor_app = make_unique<SensorApp>(server_url);
        g_sensor_app->start();
        
        // メインループ
        while (g_sensor_app->is_running()) {
            this_thread::sleep_for(chrono::seconds(1));
        }
        
        g_sensor_app->wait();
        
    } catch (const exception& e) {
        cerr << "❌ Application error: " << e.what() << endl;
        return 1;
    }
    
    cout << "✅ Application finished successfully" << endl;
    return 0;
}