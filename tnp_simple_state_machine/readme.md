# tnp_simple_state_machine

## Usage:

Simply run: `rosrun tnp_simple_state_machine tnp_simple_state_machine`

## Verbose:

Change the logger level of the package that you want to see the states messages. Example for tnp_task_manager:
* Change the loggers levels:
`rosservice call /tnp_task_manager/set_logger_level "logger: 'ros.tnp_task_manager' level: 'debug'"`
* Check if the loggers changed the level:
`rosservice call /tnp_task_manager/get_loggers`
