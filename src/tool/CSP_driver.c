#include <csp/csp.h>
#include <csp/csp_endian.h>
------------------------------------------------------------------------------------
static uint8_t node = NODE_EPS;
static uint32_t timeout = 1000;

        eps_output_set_single_req req;    // choose frame
	req.channel=channel;              // fill in the frame
	req.mode = mode;
	req.delay = csp_hton16(delay);
	return csp_transaction(CSP_PRIO_HIGH, eps_node, EPS_PORT_SET_SINGLE_OUTPUT, 0, &req, 4, NULL, 0);  // CSP send 
------------------------------------------------------------------------------------------

