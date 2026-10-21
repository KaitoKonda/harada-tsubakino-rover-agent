function subscriber = rossubscriber(topic,messageType,callback)
% Offline test double. Never add this directory to an experiment's path.
    global DRIVE_TRACE_TEST_SUBSCRIPTIONS
    assert(strcmp(messageType,'std_msgs/String'));
    subscriber = TraceTestSubscriber(callback);
    DRIVE_TRACE_TEST_SUBSCRIPTIONS{end+1} = struct('topic',string(topic), ...
        'subscriber',subscriber);
end
