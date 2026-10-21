function tests = testDriveTraceRecorder
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    testCase.TestData.tools = fullfile(root,'tools');
    testCase.TestData.helpers = fullfile(fileparts(mfilename('fullpath')),'helpers');
    addpath(testCase.TestData.tools,testCase.TestData.helpers);
end

function teardownOnce(testCase)
    rmpath(testCase.TestData.tools,testCase.TestData.helpers);
    clear global DRIVE_TRACE_TEST_SUBSCRIPTIONS
end

function testCapturesTopicsAndRawJsonWithoutSendingCommands(testCase)
    global DRIVE_TRACE_TEST_SUBSCRIPTIONS
    DRIVE_TRACE_TEST_SUBSCRIPTIONS = {};
    folder = string(tempname);
    mkdir(folder);
    folderCleanup = onCleanup(@()rmdir(folder,'s'));
    recorder = DriveTraceRecorder(["pi1","pi3"],folder);
    recorderCleanup = onCleanup(@()recorder.close());
    verifyEqual(testCase,numel(DRIVE_TRACE_TEST_SUBSCRIPTIONS),12);
    source = '{"schema_version":1,"events":[{"seq":9007199254740993}]}';
    for index = 1:12
        DRIVE_TRACE_TEST_SUBSCRIPTIONS{index}.subscriber.deliver(source);
    end
    recorder.close();
    recorder.close();
    verifyEqual(testCase,recorder.messageCount,12);
    verifyEqual(testCase,recorder.errorText,"");
    lines = splitlines(string(strtrim(fileread(recorder.logFile))));
    verifyEqual(testCase,numel(lines),12);
    for index = 1:12
        row = jsondecode(lines(index));
        verifyEqual(testCase,string(row.topic),DRIVE_TRACE_TEST_SUBSCRIPTIONS{index}.topic);
        verifyEqual(testCase,row.data,source);
        verifyTrue(testCase,endsWith(string(row.pc_received_at),"Z"));
        verifyFalse(testCase,isvalid(DRIVE_TRACE_TEST_SUBSCRIPTIONS{index}.subscriber));
    end
    clear recorderCleanup folderCleanup
end

function testInvalidNamespacesDoNotSubscribe(testCase)
    global DRIVE_TRACE_TEST_SUBSCRIPTIONS
    DRIVE_TRACE_TEST_SUBSCRIPTIONS = {};
    verifyError(testCase,@()DriveTraceRecorder("../pi1"),'DriveTrace:InvalidName');
    verifyError(testCase,@()DriveTraceRecorder(["pi1","pi1"]),'DriveTrace:DuplicateName');
    verifyEmpty(testCase,DRIVE_TRACE_TEST_SUBSCRIPTIONS);
end

function testMinimalSummariesAreRecordedWithoutDetailedEvents(testCase)
    global DRIVE_TRACE_TEST_SUBSCRIPTIONS
    DRIVE_TRACE_TEST_SUBSCRIPTIONS = {};
    folder = string(tempname);
    mkdir(folder);
    folderCleanup = onCleanup(@()rmdir(folder,'s'));
    recorder = DriveTraceRecorder("pi1",folder);
    recorderCleanup = onCleanup(@()recorder.close());
    source = '{"trace_mode":"minimal","summary_seq":1,"metrics":{"control_period_s":{"count":3,"mean":0.15}}}';
    for index = 1:numel(DRIVE_TRACE_TEST_SUBSCRIPTIONS)
        entry = DRIVE_TRACE_TEST_SUBSCRIPTIONS{index};
        if endsWith(entry.topic,"_timing")
            entry.subscriber.deliver(source);
        end
    end
    recorder.close();
    verifyEqual(testCase,recorder.messageCount,2);
    lines = splitlines(string(strtrim(fileread(recorder.logFile))));
    for index = 1:2
        row = jsondecode(lines(index));
        verifyTrue(testCase,endsWith(string(row.topic),"_timing"));
        payload = jsondecode(row.data);
        verifyEqual(testCase,string(payload.trace_mode),"minimal");
        verifyEqual(testCase,payload.metrics.control_period_s.mean,0.15);
    end
    clear recorderCleanup folderCleanup
end
