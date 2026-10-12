%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% config.m
% Global Verification Configuration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%clear;

cfg.N = 16;

cfg.WORD_LENGTH = 16;

cfg.SCALE = 4;          % RTL scales by 4

cfg.NUM_RANDOM = 20;

cfg.INPUT_FOLDER = "vectors";

cfg.OUTPUT_FOLDER = "expected";

cfg.REPORT_FOLDER = fullfile(pwd,"verification_reports");

if ~exist(cfg.INPUT_FOLDER,'dir')
    mkdir(cfg.INPUT_FOLDER);
end

if ~exist(cfg.OUTPUT_FOLDER,'dir')
    mkdir(cfg.OUTPUT_FOLDER);
end

if ~exist(cfg.REPORT_FOLDER,'dir')
    mkdir(cfg.REPORT_FOLDER);
end
