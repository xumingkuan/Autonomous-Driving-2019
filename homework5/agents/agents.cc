// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework5/agents/agents.h"

#include "homework5/agents/sample/sample_agent.h"
#include "homework5/agents/xumingkuan/xumingkuan_agent.h"

// Register sample vehicle agent to a factory with its type name "sample_agent"
static simulation::Registrar<::sample::SampleVehicleAgent> registrar("sample_agent");

static simulation::Registrar<::xumingkuan::XmkVehicleAgent>
    registrar_xumingkuan("xumingkuan_agent");

