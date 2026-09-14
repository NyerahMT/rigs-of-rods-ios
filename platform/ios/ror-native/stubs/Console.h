#pragma once

#include <string>

namespace RoR {

// RigDef parsing and SequentialImporter only need the console message API.
// iOS owns presentation/log routing; keep the parser's desktop dependency out
// of the simulation library without changing any parsing or node-resolution
// behavior.
class Console
{
public:
    enum MessageType
    {
        CONSOLE_HELP,
        CONSOLE_TITLE,
        CONSOLE_SYSTEM_NOTICE,
        CONSOLE_SYSTEM_ERROR,
        CONSOLE_SYSTEM_WARNING,
        CONSOLE_SYSTEM_REPLY,
        CONSOLE_SYSTEM_NETCHAT
    };

    enum MessageArea
    {
        CONSOLE_MSGTYPE_INFO,
        CONSOLE_MSGTYPE_LOG,
        CONSOLE_MSGTYPE_SCRIPT,
        CONSOLE_MSGTYPE_ACTOR,
        CONSOLE_MSGTYPE_TERRN
    };

    void putMessage(MessageArea, MessageType, const std::string&, std::string = std::string()) {}
};

} // namespace RoR
