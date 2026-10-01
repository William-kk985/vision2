#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

namespace tools
{
class Exiter
{
public:
  Exiter();

  bool exit() const;

  /// ⭐ W18：编程式请求退出（供热键 `q` 用；原来只能靠 SIGINT）
  static void request_exit();

  /// ⭐ W19：静态查询（不需要实例；`Exiter` 的构造有单例限制）
  static bool exit_requested();
};

}  // namespace tools

#endif  // TOOLS__EXITER_HPP