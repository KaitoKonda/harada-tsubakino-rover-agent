function logFile = recordDriveTrace(vehicleNames, durationSeconds, outputDirectory)
% Timed standalone capture. For a same-session experiment use DriveTraceRecorder.
% Connect to the ROS master with rosinit first. Sends no vehicle commands.
    arguments
        vehicleNames (1,:) string
        durationSeconds (1,1) double {mustBeFinite,mustBePositive} = 60
        outputDirectory (1,1) string = fullfile(pwd,"logs")
    end
    recorder = DriveTraceRecorder(vehicleNames,outputDirectory);
    cleanup = onCleanup(@()recorder.close());
    logFile = recorder.logFile;
    fprintf('Recording for %.1f seconds. Start the rover Launch now for startup events.\n', ...
        durationSeconds);
    clock = tic;
    while toc(clock) < durationSeconds && strlength(recorder.errorText) == 0
        pause(min(0.1,max(0,durationSeconds-toc(clock))));
    end
    clear cleanup
    fprintf('Saved %d ROS messages: %s\n',recorder.messageCount,logFile);
    if recorder.messageCount == 0
        warning('DriveTrace:NoMessages', ...
            'No telemetry received. Check launch mode and trace_enabled.');
    end
end
