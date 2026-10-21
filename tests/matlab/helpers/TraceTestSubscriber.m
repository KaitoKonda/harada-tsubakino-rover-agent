classdef TraceTestSubscriber < handle
    properties
        callback
    end
    methods
        function obj = TraceTestSubscriber(callback)
            obj.callback = callback;
        end
        function deliver(obj,data)
            obj.callback([],struct('Data',data));
        end
    end
end
