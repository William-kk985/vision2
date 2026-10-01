#include "expense.hpp"

#include <algorithm>
#include <vector>

namespace tools
{

namespace
{
std::string fmt_items(const std::unordered_map<std::string, int64_t> & m)
{
  std::vector<std::pair<std::string, int64_t>> items(m.begin(), m.end());
  std::sort(items.begin(), items.end(),
            [](const auto & a, const auto & b) { return a.second > b.second; });

  std::string out;
  for (const auto & [tag, us] : items) {
    if (!out.empty()) out += "  ";
    out += tag + "=" + std::to_string(us) + "us";
  }
  return out.empty() ? "(empty)" : out;
}
}  // namespace

std::string Expense::summary() const { return fmt_items(us_); }
std::string Expense::last_summary() const { return fmt_items(last_); }

}  // namespace tools
