// Timeout for each ping attempt
const int timeout_ms = 100;

// Number of ping attempts
const uint8_t attempts = 1;

// Spin period
const unsigned int spin_timeout = RCL_MS_TO_NS(100);

// Enum with connection status
enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

while (true)
{
    switch (state)
    {
    case WAITING_AGENT:
        // Check for agent connection
        state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
        break;

    case AGENT_AVAILABLE:
        // Create micro-ROS entities
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

        if (state == WAITING_AGENT)
        {
            // Creation failed, release allocated resources
            destroy_entities();
        };
        break;

    case AGENT_CONNECTED:
        // Check connection and spin on success
        state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, spin_timeout);
        }
        break;

    case AGENT_DISCONNECTED:
        // Connection is lost, destroy entities and go back to first step
        destroy_entities();
        state = WAITING_AGENT;
        break;

    default:
        break;
    }
}