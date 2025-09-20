/**
 * @file uart_driver.hpp
 * @brief UART串口驱动头文件
 * @author RM Team
 * @date 2025
 * 
 * 该文件提供了基于 Boost.Asio 的高性能异步串口通信实现。
 * 支持多线程异步读写、自动错误处理和恢复、线程安全的消息队列等功能。
 * 
 * 依赖库：
 * - Boost.Asio: 用于异步I/O操作
 * - C++11及以上标准库
 */

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <deque>
#include <memory>
#include <atomic>
#include <functional>
#include <chrono>

// Boost.Asio includes
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/system/error_code.hpp>

# ifndef UART_DRIVER_HPP
# define UART_DRIVER_HPP

/**
 * @namespace UARTDriver
 * @brief UART串口驱动命名空间
 * 
 * 该命名空间包含了用于串口通信的所有相关类和函数。
 * 提供异步、线程安全的串口通信功能，适用于高性能的串口数据交互场景。
 */
namespace RMManager {

/**
 * @brief 串口通信类
 * 
 * SerialCommunicator 提供了一个高性能的异步串口通信接口，基于 Boost.Asio 实现。
 * 
 * 主要特性：
 * - 异步读写操作，不阻塞主线程
 * - 多线程支持，自动管理工作线程池
 * - 线程安全的写入队列
 * - 自动重连和错误处理
 * - 可配置的读取缓冲区大小
 * - 基于回调的数据接收机制
 * 
 * 使用示例：
 * @code
 * UARTDriver::SerialCommunicator serial("/dev/ttyUSB0", 115200);
 * serial.register_read_callback([](const std::vector<uint8_t>& data) {
 *     // 处理接收到的数据
 * });
 * serial.openPort();
 * serial.startRead();
 * serial.writeData({0x01, 0x02, 0x03});
 * @endcode
 */
class SerialCommunicator {
public:
    using ReadCallback = std::function<void(const std::vector<uint8_t>&)>;

    /**
     * @brief 构造函数：初始化串口通信器
     * @param port_name 串口设备名称（如 /dev/ttyUSB0 或 COM1）
     * @param baud_rate 波特率（如 9600, 115200 等）
     * @param read_buffer_size 读取缓冲区大小，默认为512字节
     * 
     * 注意：构造函数不会自动打开串口连接，需要手动调用 openPort() 方法
     * 会自动创建多个工作线程来处理异步I/O操作
     */
    SerialCommunicator(const std::string& port_name, unsigned int baud_rate, unsigned int read_buffer_size = 512)
        : io_context_(),
          serial_port_(io_context_),
          read_buffer_(read_buffer_size),
          write_in_progress_(false),
          work_guard_(boost::asio::make_work_guard(io_context_)),
          port_name_(port_name),
          baud_rate_(baud_rate)
    {
        // 启动 io_context 工作线程池
        unsigned int num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) num_threads = 1;
        std::cout << "Starting " << num_threads << " io_context worker threads." << std::endl;

        for (unsigned int i = 0; i < num_threads; ++i) {
            io_threads_.emplace_back([this]() {
                io_context_.run(); // 每个线程都运行 io_context 的事件循环
                std::cout << "io_context worker thread exited." << std::endl;
            });
        }
    }

    /**
     * @brief 析构函数：安全清理资源
     * 
     * 执行以下清理操作：
     * 1. 停止所有异步读写操作
     * 2. 停止 io_context 事件循环
     * 3. 等待所有工作线程安全退出
     * 4. 关闭串口连接
     * 5. 清理所有资源
     */
    ~SerialCommunicator() {
        // 在析构函数中确保 io_context_ 停止且线程被加入
        stopAllActivity(); // 停止所有读写，并清空队列
        io_context_.stop(); // 停止 io_context，让 io_threads_.run() 返回

        for (auto& t : io_threads_) {
            if (t.joinable()) {
                t.join(); // 等待所有 io_context 线程退出
            }
        }
        io_threads_.clear(); // 清空线程向量

        // 最后关闭物理串口
        if (serial_port_.is_open()) {
            boost::system::error_code ec;
            serial_port_.close(ec);
            if (ec) {
                std::cerr << "Error closing serial port in destructor: " << ec.message() << std::endl;
            } else {
                std::cout << "Serial port closed in destructor." << std::endl;
            }
        }
        std::cout << "SerialCommunicator destroyed." << std::endl;
    }

    /**
     * @brief 检查串口是否处于正常工作状态
     * @return true 如果串口已打开且没有发生错误；false 否则
     * 
     * 该函数检查两个条件：
     * 1. 串口是否已经打开
     * 2. 是否没有发生错误（error_occurred_ 标志）
     */
    bool isPortOK() const {
        return serial_port_.is_open() && !error_occurred_.load();
    }

    /**
     * @brief 打开串口连接
     * @return true 成功打开串口；false 打开失败
     * 
     * 该函数执行以下操作：
     * 1. 检查串口是否已经打开
     * 2. 尝试打开指定的串口设备
     * 3. 配置串口参数（波特率、数据位、停止位、校验位、流控制）
     * 4. 设置错误状态标志
     */
    bool openPort() {
        if (serial_port_.is_open()) {
            std::cerr << "Serial port is already open." << std::endl;
            return false;
        }

        boost::system::error_code ec;
        serial_port_.open(port_name_, ec);
        if (ec) {
            std::cerr << "Failed to open serial port " << port_name_ << ": " << ec.message() << std::endl;
            error_occurred_.store(true); // 标记错误
            return false;
        }

        try {
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        } catch (const boost::system::system_error& ex) {
            std::cerr << "Failed to set serial port options for " << port_name_ << ": " << ex.what() << std::endl;
            serial_port_.close(ec); // 关闭失败的端口
            error_occurred_.store(true); // 标记错误
            return false;
        }

        error_occurred_.store(false); // 成功打开，清除错误标记
        std::cout << "Serial port " << port_name_ << " opened at " << baud_rate_ << " baud." << std::endl;
        return true;
    }

    /**
     * @brief 关闭串口连接
     * @return true 成功关闭串口；false 关闭失败
     * 
     * 该函数执行以下操作：
     * 1. 检查串口是否已经打开
     * 2. 停止所有异步读写操作
     * 3. 关闭串口设备
     * 4. 更新错误状态标志
     */
    bool closePort() {
        if (!serial_port_.is_open()) {
            std::cout << "Serial port is not open, nothing to close." << std::endl;
            return true;
        }

        // 取消所有挂起的异步操作
        stopAllActivity(); // 停止所有读写，并清空队列

        boost::system::error_code ec;
        serial_port_.close(ec);
        if (ec) {
            std::cerr << "Error closing serial port " << port_name_ << ": " << ec.message() << std::endl;
            error_occurred_.store(true); // 标记错误
            return false;
        }
        std::cout << "Serial port " << port_name_ << " closed." << std::endl;
        error_occurred_.store(false); // 成功关闭，清除错误标记
        return true;
    }

    /**
     * @brief 重新打开串口连接
     * @return true 成功重新打开串口；false 重新打开失败
     * 
     * 该函数执行以下操作：
     * 1. 先关闭当前的串口连接
     * 2. 等待100毫秒确保端口完全释放
     * 3. 重新打开串口连接
     * 
     * 通常用于串口通信出现问题时的恢复操作
     */
    bool reopenPort() {
        if (!closePort()) {
            std::cerr << "Failed to close existing port before reopening." << std::endl;
            return false;
        }
        // 需要一些时间确保端口完全释放，尤其在Windows上
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return openPort();
    }

    /**
     * @brief 注册数据接收回调函数
     * @param callback 当接收到数据时调用的回调函数，参数为接收到的数据向量
     * 
     * 当串口接收到数据时，会自动调用注册的回调函数。
     * 回调函数的参数是一个 std::vector<uint8_t>，包含接收到的字节数据。
     * 注意：回调函数在工作线程中执行，需要注意线程安全。
     */
    void register_read_callback(ReadCallback callback) {
        read_callback_ = std::move(callback);
    }

    /**
     * @brief 开始异步读取串口数据
     * @return true 成功启动读取操作；false 启动失败
     * 
     * 该函数启动异步读取操作，当有数据到达时会自动调用注册的回调函数。
     * 读取操作是持续的，会一直进行直到调用 stopRead() 或关闭串口。
     * 
     * 注意：必须先调用 register_read_callback() 注册回调函数，
     * 并且串口必须处于正常工作状态。
     */
    bool startRead() {
        if (!isPortOK()) {
            std::cerr << "Cannot start read: Serial port not open or in bad state." << std::endl;
            return false;
        }
        io_context_.post([this]() {
            _do_startRead();
        });
        return true;
    }

    /**
     * @brief 停止异步读取操作
     * @return true 操作执行成功（不保证立即停止）
     * 
     * 该函数取消当前进行的异步读取操作。
     * 注意：由于异步操作的特性，调用此函数后读取操作可能不会立即停止，
     * 但会在当前读取操作完成后不再启动新的读取操作。
     */
    bool stopRead() {
        boost::system::error_code ec;
        serial_port_.cancel(ec);
        if (ec && ec != boost::asio::error::operation_not_supported &&
                   ec != boost::asio::error::bad_descriptor) {
             std::cerr << "Error cancelling serial port operations: " << ec.message() << std::endl;
             error_occurred_.store(true); // 标记错误
        } else if (!ec) {
             std::cout << "Serial read/write operations cancelled." << std::endl;
        }
        return true;
    }

    /**
     * @brief 异步发送数据到串口
     * @param data 要发送的字节数据向量
     * 
     * 该函数将数据加入发送队列，并异步发送。
     * 发送操作是顺序执行的，确保数据按照调用顺序发送。
     * 
     * 注意：
     * - 如果串口未打开或处于错误状态，会输出错误信息但不会抛出异常
     * - 空数据向量会被忽略
     * - 函数立即返回，不会阻塞等待发送完成
     */
    void writeData(const std::vector<uint8_t>& data) {
        if (!isPortOK()) {
            std::cerr << "Cannot write: Serial port not open or in bad state." << std::endl;
            return;
        }
        if (data.empty()) return;

        auto shared_data = std::make_shared<std::vector<uint8_t>>(data);

        {
            std::lock_guard<std::mutex> lock(write_queue_mutex_);
            write_queue_.push_back(shared_data);
        }
        io_context_.post(boost::bind(&SerialCommunicator::_do_write_next_message, this));
    }

private:
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    std::vector<uint8_t> read_buffer_;
    std::deque<std::shared_ptr<std::vector<uint8_t>>> write_queue_;
    std::mutex write_queue_mutex_;
    // 尽管 write_in_progress_ 也会被锁保护，但atomic可以提供更多可见性
    std::atomic<bool> write_in_progress_{false}; 
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::vector<std::thread> io_threads_;
    ReadCallback read_callback_ = nullptr; // 明确初始化为空

    std::string port_name_;
    unsigned int baud_rate_;
    std::atomic<bool> error_occurred_{false}; // 用于跟踪串口是否遇到错误

    /**
     * @brief 停止所有读写活动并清空队列
     * 
     * 该函数执行以下操作：
     * 1. 取消所有异步读取操作
     * 2. 清空写入队列中的待发送数据
     * 3. 重置写入进行中的状态标志
     * 
     * 通常在关闭串口或析构时调用，确保所有异步操作被正确清理。
     */
    void stopAllActivity() {
        stopRead(); // 取消异步读取

        { // 锁住写入队列，清空并重置状态
            std::lock_guard<std::mutex> lock(write_queue_mutex_);
            write_queue_.clear();
            write_in_progress_ = false;
        }
    }

    /**
     * @brief 内部函数：实际启动异步读取操作
     * 
     * 该函数在 io_context 工作线程中执行，负责：
     * 1. 检查串口状态是否正常
     * 2. 启动 async_read_some 异步读取操作
     * 3. 设置读取完成后的回调处理函数
     * 
     * 注意：此函数仅供内部调用，不应直接从外部调用。
     */
    void _do_startRead() {
        // 只有当端口打开且没有错误时才启动读取
        if (!isPortOK()) {
            std::cerr << "Internal: Skipping read start as port is not OK." << std::endl;
            return;
        }
        serial_port_.async_read_some(
            boost::asio::buffer(read_buffer_),
            boost::bind(&SerialCommunicator::_handle_read, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred)
        );
    }

    /**
     * @brief 异步读取操作完成后的回调处理函数
     * @param error 异步操作的错误码
     * @param bytes_transferred 实际读取的字节数
     * 
     * 该函数处理异步读取的结果：
     * 1. 如果读取成功，调用用户注册的回调函数处理数据
     * 2. 如果读取成功，启动下一次异步读取操作（持续读取）
     * 3. 如果读取失败，根据错误类型进行相应处理和错误标记
     * 
     * 注意：此函数在 io_context 工作线程中执行。
     */
    void _handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
        if (!error) {
            if (read_callback_) {
                std::vector<uint8_t> received_data(read_buffer_.begin(), read_buffer_.begin() + bytes_transferred);
                read_callback_(received_data);
            }
            _do_startRead(); // 继续启动下一次异步读取
        } else {
            if (error == boost::asio::error::operation_aborted) {
                std::cout << "Read operation aborted (port closed or stopped)." << std::endl;
            } else {
                std::cerr << "Error during read: " << error.message() << std::endl;
                error_occurred_.store(true); // 标记错误
            }
        }
    }

    /**
     * @brief 内部函数：启动下一个异步写入操作
     * 
     * 该函数处理写入队列中的消息：
     * 1. 检查串口状态是否正常
     * 2. 检查写入队列是否有待发送的消息
     * 3. 检查当前是否没有写入操作正在进行
     * 4. 如果条件满足，启动 async_write_some 异步写入操作
     * 
     * 写入操作是顺序执行的，确保数据按照队列顺序发送。
     * 注意：此函数仅供内部调用，不应直接从外部调用。
     */
    void _do_write_next_message() {
        // 只有当端口打开且没有错误时才尝试写入
        if (!isPortOK()) {
            std::cerr << "Internal: Skipping write start as port is not OK." << std::endl;
            {
                std::lock_guard<std::mutex> lock(write_queue_mutex_);
                write_queue_.clear(); // 清空队列，因为端口不可用
                write_in_progress_ = false;
            }
            return;
        }

        std::lock_guard<std::mutex> lock(write_queue_mutex_);
        if (!write_queue_.empty() && !write_in_progress_) {
            write_in_progress_ = true;
            auto message_holder = write_queue_.front();

            serial_port_.async_write_some(
                boost::asio::buffer(*message_holder),
                boost::bind(&SerialCommunicator::_handle_write, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred,
                            message_holder)
            );
        }
    }

    /**
     * @brief 异步写入操作完成后的回调处理函数
     * @param error 异步操作的错误码
     * @param bytes_transferred 实际发送的字节数
     * @param message_holder 发送消息的智能指针（用于保持数据生命周期）
     * 
     * 该函数处理异步写入的结果：
     * 1. 重置写入进行中的状态标志
     * 2. 根据写入结果进行相应处理（成功或失败）
     * 3. 从写入队列中移除已处理的消息
     * 4. 尝试启动下一个写入操作
     * 
     * 注意：此函数在 io_context 工作线程中执行，使用互斥锁保证线程安全。
     */
    void _handle_write(const boost::system::error_code& error, size_t bytes_transferred,
                      std::shared_ptr<std::vector<uint8_t>> message_holder) {
        std::lock_guard<std::mutex> lock(write_queue_mutex_);
        write_in_progress_ = false;

        if (!error) {
            // std::cout << "Sent " << bytes_transferred << " bytes." << std::endl;
            write_queue_.pop_front();
        } else {
            if (error == boost::asio::error::operation_aborted) {
                // std::cerr << "Write operation aborted (port closed or stopped)." << std::endl;
            } else {
                std::cerr << "Error during write: " << error.message() << std::endl;
                error_occurred_.store(true); // 标记错误
            }
            write_queue_.pop_front();
        }
        _do_write_next_message(); // 尝试发送队列中的下一个消息
    }
};


} // namespace RMManager

# endif // UART_DRIVER_HPP