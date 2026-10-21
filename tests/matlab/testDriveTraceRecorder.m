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
    verifyEqual(testCase,numel(DRIVE_TRACE_TEST_SUBSCRIPTIONS),8);
    source = '{"schema_version":1,"events":[{"seq":9007199254740993}]}';
    for index = 1:8
        DRIVE_TRACE_TEST_SUBSCRIPTIONS{index}.subscriber.deliver(source);
    end
    recorder.close();
    recorder.close();
    verifyEqual(testCase,recorder.messageCount,8);
    verifyEqual(testCase,recorder.errorText,"");
    lines = splitlines(string(strtrim(fileread(recorder.logFile))));
    verifyEqual(testCase,numel(lines),8);
    for index = 1:8
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
