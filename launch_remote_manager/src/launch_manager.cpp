#ifdef _WIN32
#include <windows.h>
#include <Lmcons.h>
#include <locale>
#include <codecvt>
#endif

#ifdef __unix__
#include <unistd.h>
#include <limits.h>
#endif

#include <string>
#include <tuple>
#include <rclcpp/rclcpp.hpp>

class LaunchManager : public rclcpp::Node
{
public:
  LaunchManager(const std::string & machine, const std::string & user)
  : Node("launch_manager", machine + "/" + user)
  {
  }
};

std::tuple<std::string, std::string> get_machine_and_user()
{
  // https://stackoverflow.com/questions/8666378/detect-windows-or-linux-in-c-c
  // https://stackoverflow.com/questions/27914311/get-computer-name-and-logged-user-name

  #ifdef _WIN32
  // TODO(anyone) - this Windows code is currently untested

  // https://stackoverflow.com/questions/4721429/correct-usage-of-getcomputername-do-i-need-to-reserve-extra-space-for-null-cha
  TCHAR machine_name[MAX_COMPUTERNAME_LENGTH + 1];
  DWORD machine_name_size = MAX_COMPUTERNAME_LENGTH + 1;
  GetComputerNameA(machine_name, &machine_name_size);

  // https://stackoverflow.com/questions/11587426/get-current-username-in-c-on-windows
  TCHAR user_name[UNLEN + 1];
  DWORD user_name_size = UNLEN + 1;
  GetUserNameA(user_name, &user_name_size);

  // https://stackoverflow.com/questions/6291458/how-to-convert-a-tchar-array-to-stdstring
  #ifndef UNICODE
    std::string machine {machine_name};
    std::string user {user_name};
  #else
    std::wstring wmachine {machine_name};
    std::wstring wuser {user_name};
    // https://stackoverflow.com/questions/4804298/how-to-convert-wstring-into-string/4804506#4804506
    std::wstring_convert<std::codecvt_utf8<wchar_t>, wchar_t> converter;
    std::string machine = converter.to_bytes(wmachine);
    std::string user = converter.to_bytes(wuser);
  #endif

  #endif

  #ifdef __unix__
    char machine_name[HOST_NAME_MAX];
    char user_name[LOGIN_NAME_MAX];

    gethostname(machine_name, HOST_NAME_MAX);
    getlogin_r(user_name, LOGIN_NAME_MAX);

    std::string machine {machine_name};
    std::string user {user_name};
  #endif

  return {machine, user};
}

int main(int argc, char ** argv)
{
  // Get machine and user names for namespacing
  // TODO(nmorales) consider removing username
  const auto [machine, user] = get_machine_and_user();

  rclcpp::init(argc, argv);
  // TODO(nmorales) consider multithreaded executor. Probably not desired though
  rclcpp::spin(std::make_shared<LaunchManager>(machine, user));
  rclcpp::shutdown();

  return 0;
}