function log = read_usb_log(logDir)
%READ_USB_LOG Load USB logger CSV/MAT files into MATLAB.
%
% Usage:
%   log = read_usb_log(".")
%   log = read_usb_log("C:\\path\\to\\log")
%
% The script expects the Python logger output files in the directory:
%   - imu.csv
%   - robot_motion.csv
%   - solved_rc_cmd.csv (optional, only if the MCU sends 0x0C)
%   - usb_log.mat (optional, if exported)
%
% Returned data:
%   log.imu, log.robot_motion, log.solved_rc_cmd: tables
%   log.meta: folder and source information
%
% If usb_log.mat exists, this function also loads the MAT file and exposes it
% in log.mat for convenience.

    if nargin < 1 || strlength(string(logDir)) == 0
        logDir = pwd;
    end
    logDir = char(logDir);

    log = struct();
    log.meta = struct('folder', logDir);

    imuFile = fullfile(logDir, 'imu.csv');
    motionFile = fullfile(logDir, 'robot_motion.csv');
    solvedFile = fullfile(logDir, 'solved_rc_cmd.csv');
    matFile = fullfile(logDir, 'usb_log.mat');

    if isfile(imuFile)
        log.imu = readtable(imuFile, 'VariableNamingRule', 'preserve');
    else
        log.imu = table();
    end

    if isfile(motionFile)
        log.robot_motion = readtable(motionFile, 'VariableNamingRule', 'preserve');
    else
        log.robot_motion = table();
    end

    if isfile(solvedFile)
        log.solved_rc_cmd = readtable(solvedFile, 'VariableNamingRule', 'preserve');
    else
        log.solved_rc_cmd = table();
    end

    if isfile(matFile)
        log.mat = load(matFile);
    else
        log.mat = struct();
    end
end
