#ifndef DEFAULT_APP_HMI_HPP
#define DEFAULT_APP_HMI_HPP

#include "manymove_hmi/app_module.hpp"

/*  Application-specific module with the *original* 10 keys ----------
 *  Nothing here except the list-builder.                             */
class DefaultAppModule : public AppModule
{
public:
  explicit DefaultAppModule(QWidget * parent = nullptr)
  : AppModule(buildKeys(), parent) {}

private:
  static std::vector<KeyConfig> buildKeys();
};

#endif   /* DEFAULT_APP_HMI_HPP */
