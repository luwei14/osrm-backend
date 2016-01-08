#include "util/datastore_options.hpp"
#include "datastore/datastore.hpp"
#include "util/osrm_exception.hpp"
#include "util/simple_logger.hpp"
#include "util/typedefs.hpp"

// FIXME remove after move to datastore
using namespace osrm::datastore;
using namespace osrm;

int main(const int argc, const char *argv[]) try
{
    util::LogPolicy::GetInstance().Unmute();

    DataPaths paths;
    if (!util::GenerateDataStoreOptions(argc, argv, paths))
    {
        return EXIT_SUCCESS;
    }

    Datastore store(paths);
    return store.Run();
}
catch (const std::bad_alloc &e)
{
    util::SimpleLogger().Write(logWARNING) << "[exception] " << e.what();
    util::SimpleLogger().Write(logWARNING)
        << "Please provide more memory or disable locking the virtual "
           "address space (note: this makes OSRM swap, i.e. slow)";
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    util::SimpleLogger().Write(logWARNING) << "caught exception: " << e.what();
}
