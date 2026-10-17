"""Explicit ACK for USV selection before sending a mission X command."""
import argparse
import json
import sys

MODES = ('linear', 'figure_eight', 'variable_speed_eight',
         'variable_speed_s', 'variable_speed_z')


def canonical_mode(value):
    """Normalize a number or the public trajectory parameter name."""
    value = str(value).strip().lower()
    if value in tuple(str(i) for i in range(1, 6)):
        return MODES[int(value)-1]
    if value not in MODES:
        raise ValueError('INVALID_MOTION_MODE')
    return value


def validate_ack(successful, reason, requested):
    """Do not equate service discovery or a different selection with success."""
    if not successful:
        raise ValueError('MODE_SELECTION_REJECTED: '+str(reason))
    try:
        result = json.loads(reason)
        mode = canonical_mode(result['mode'])
    except (ValueError, TypeError, KeyError) as exc:
        raise ValueError('INVALID_MODE_ACK') from exc
    if mode != canonical_mode(requested):
        raise ValueError('MODE_ACK_MISMATCH')
    return dict(result, mode=mode)


def select_mode(mode=None, timeout=3.):
    """Read-and-confirm current mode when old automated scripts send X directly."""
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    rclpy.init()
    node = rclpy.create_node('usv_motion_mode_console')
    try:
        def call(service, name, request):
            client = node.create_client(service, '/moving_target/'+name)
            if not client.wait_for_service(timeout_sec=timeout):
                raise ValueError('MODE_SERVICE_UNAVAILABLE')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done() or future.exception():
                raise ValueError('MODE_ACK_TIMEOUT')
            return future.result()
        if mode is None:
            reply = call(GetParameters, 'get_parameters',
                         GetParameters.Request(names=['trajectory_type']))
            if len(reply.values) != 1:
                raise ValueError('MODE_STATUS_UNAVAILABLE')
            mode = reply.values[0].string_value
        mode = canonical_mode(mode)
        parameter = Parameter(name='trajectory_type', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=mode))
        reply = call(SetParameters, 'set_parameters',
                     SetParameters.Request(parameters=[parameter]))
        if len(reply.results) != 1:
            raise ValueError('INVALID_MODE_ACK')
        result = reply.results[0]
        return validate_ack(result.successful, result.reason, mode)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    """Return nonzero on every missing/rejected/incorrect acknowledgement."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', nargs='?', default=None)
    parser.add_argument('--timeout', type=float, default=3.)
    args = parser.parse_args()
    try:
        print(json.dumps(select_mode(args.mode, args.timeout), allow_nan=False))
        return 0
    except (ValueError, RuntimeError) as exc:
        print(json.dumps(dict(ack=False, reason=str(exc))), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
