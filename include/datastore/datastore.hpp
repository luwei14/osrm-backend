#ifndef DATASTORE_DATASTORE_HPP
#define DATASTORE_DATASTORE_HPP

#include <boost/filesystem.hpp>

#include <unordered_map>

namespace osrm
{
namespace datastore
{
using DataPaths = std::unordered_map<std::string, boost::filesystem::path>;
class Datastore
{
public:
    Datastore(const DataPaths& data_paths);
    int Run();
private:
    DataPaths paths;
};
}
}

#endif
