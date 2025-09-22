#include <iostream>
#include <thread>                //多线程相关头文件
#include <chrono>                //时间相关头文件
#include <functional>            //函数包装器
#include "cpp-httplib/httplib.h" //下载相关头文件
using namespace std;

class Download
{
public:
    void download(const string &host, const string &path,
                  const function<void(const string &, const string)> &callback_word_count)
    {
        cout << "" << this_thread::get_id() << endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200)
        {
            callback_word_count(path, response->body);
        }
    };
    void star_download(const string &host, const string &path,
                       const function<void(const string &, const string)> &callback_word_count)
    {
        auto download_fun = bind(&Download::download, this,
                                 placeholders::_1, placeholders::_2, placeholders::_3); // bind(函数模板地址，对象指针，占位符（根据函数变量而定）)
        thread thread(download_fun, host, path, callback_word_count);
        thread.detach();
    };
};

int main()
{
    auto d = Download();
    auto word_count = [](const string &path, const string &result) -> void
    {
        cout << "下在完成" << path << ":" << result.length() << "->" << result.substr(0, 9) << endl;
    };
    d.star_download("http://0.0.0.0:8000", "/novel1.txt", word_count);
    d.star_download("http://0.0.0.0:8000", "/novel2.txt", word_count);
    d.star_download("http://0.0.0.0:8000", "/novel3.txt", word_count);
    this_thread::sleep_for(chrono::milliseconds(1000 * 10)); // 休眠10s

    return 0;
}