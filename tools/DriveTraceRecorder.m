classdef DriveTraceRecorder < handle
% Asynchronous PC-side recording; keeps recording while runControlStation runs.
% Connect with rosinit, then retain this object until recording is finished.
    properties (SetAccess = private)
        logFile (1,1) string
        messageCount (1,1) double = 0
        errorText (1,1) string = ""
    end
    properties (Access = private)
        fid (1,1) double = -1
        subscribers cell = {}
        closed (1,1) logical = false
    end
    methods
        function obj = DriveTraceRecorder(vehicleNames, outputDirectory)
            arguments
                vehicleNames (1,:) string
                outputDirectory (1,1) string = fullfile(pwd,"logs")
            end
            assert(~isempty(vehicleNames) && all(matches(vehicleNames, ...
                regexpPattern("^[a-z][a-z0-9_]*$"))), ...
                'DriveTrace:InvalidName','Use namespaces such as pi1 or pi3.');
            assert(numel(unique(vehicleNames)) == numel(vehicleNames), ...
                'DriveTrace:DuplicateName','Vehicle namespaces must be unique.');
            if ~isfolder(outputDirectory), mkdir(outputDirectory); end
            obj.logFile = string(tempname(outputDirectory)) + "-drive-trace.jsonl";
            obj.fid = fopen(obj.logFile,'w','n','UTF-8');
            assert(obj.fid >= 0,'DriveTrace:OpenFailed', ...
                'Could not open %s.',obj.logFile);
            streams = ["controller","controller_metadata","wheel","wheel_metadata"];
            try
                for name = vehicleNames
                    for stream = streams
                        topic = "/" + name + "/drive_trace/" + stream;
                        obj.subscribers{end+1} = rossubscriber(char(topic), ...
                            'std_msgs/String',@(~,message) obj.record(topic,message));
                    end
                end
            catch exception
                obj.close();
                rethrow(exception)
            end
            fprintf('Drive telemetry recording started: %s\n',obj.logFile);
        end

        function close(obj)
            if obj.closed, return; end
            obj.closed = true;
            for index = 1:numel(obj.subscribers)
                delete(obj.subscribers{index});
            end
            obj.subscribers = {};
            if obj.fid >= 0
                fclose(obj.fid);
                obj.fid = -1;
            end
        end

        function delete(obj)
            obj.close();
        end
    end
    methods (Access = private)
        function record(obj,topic,message)
            if obj.closed, return; end
            try
                receivedAt = datetime('now','TimeZone','UTC', ...
                    'Format',"yyyy-MM-dd'T'HH:mm:ss.SSSXXX");
                % Preserve original JSON; decode after recording has finished.
                row = struct('topic',topic,'pc_received_at',string(receivedAt), ...
                    'data',string(message.Data));
                count = fprintf(obj.fid,'%s\n',jsonencode(row));
                assert(count > 0,'DriveTrace:WriteFailed','Could not write telemetry.');
                obj.messageCount = obj.messageCount + 1;
            catch exception
                obj.errorText = string(exception.message);
                obj.close();
                warning('DriveTrace:RecordingStopped','Telemetry recording stopped: %s', ...
                    obj.errorText);
            end
        end
    end
end
