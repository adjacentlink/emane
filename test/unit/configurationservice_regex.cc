#include "configurationservice.h"
#include "emane/registrarexception.h"

#include <cstdlib>
#include <iostream>
#include <string>

void check(bool condition, const char * message)
{
  if(!condition)
    {
      std::cerr << "FAIL: " << message << '\n';
      std::exit(EXIT_FAILURE);
    }
}

int main()
{
  using namespace EMANE;

  auto * service = ConfigurationService::instance();

  constexpr BuildId buildId = 999999;

  struct DummyMutable : RunningStateMutable {};
  DummyMutable component;
  service->registerRunningStateMutable(buildId, &component);

  // Invalid regex must be rejected during registration.
  bool invalidRejected = false;

  try
    {
      service->registerNonNumericAny(buildId,
                                     "invalid",
                                     Any::Type::TYPE_STRING,
                                     {},
                                     {},
                                     "",
                                     1,
                                     1,
                                     "[");
    }
  catch(const RegistrarException &)
    {
      invalidRejected = true;
    }

  check(invalidRejected, "Invalid regex was accepted");

  // Register a pattern that also permits partial matches.
  service->registerNonNumericAny(buildId,
                                 "pattern",
                                 Any::Type::TYPE_STRING,
                                 ConfigurationProperties::MODIFIABLE,
                                 {},
                                 "",
                                 1,
                                 1,
                                 "abc");

  // buildUpdates(): partial match succeeds.
  auto updates = service->buildUpdates(
    buildId, {{"pattern", {"prefix-abc-suffix"}}});

  check(updates.size() == 1, "Unexpected number of configuration updates");

  // buildUpdates(): mismatch throws.
  bool buildRejected = false;

  try
    {
      service->buildUpdates(buildId, {{"pattern", {"xyz"}}});
    }
  catch(const ConfigurationException &)
    {
      buildRejected = true;
    }

  check(buildRejected, "buildUpdates accepted a regex mismatch");

  // update(): partial match succeeds.
  service->update(
    buildId, {{"pattern", {Any{std::string{"prefix-abc-suffix"}}}}});

  // update(): mismatch throws.
  bool updateRejected = false;

  try
    {
      service->update(
        buildId, {{"pattern", {Any{std::string{"xyz"}}}}});
    }
  catch(const ConfigurationException &)
    {
      updateRejected = true;
    }

  check(updateRejected, "update accepted a regex mismatch");

  std::cout << "EMANE regex regression tests passed\n";
}
