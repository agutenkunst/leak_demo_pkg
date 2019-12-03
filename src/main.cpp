#include <pluginlib/class_loader.h>
#include <leak_demo_pkg/plugin_base.h>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<plugin_base::Base> poly_loader("leak_demo_pkg", "plugin_base::Base");

  try
  {
    boost::shared_ptr<plugin_base::Base> plugin = poly_loader.createInstance("plugin::Plugin");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}