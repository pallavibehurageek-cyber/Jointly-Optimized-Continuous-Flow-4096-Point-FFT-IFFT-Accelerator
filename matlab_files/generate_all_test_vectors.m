%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% generate_all_vectors.m
%
% Purpose:
%   Generates all verification vectors for BS16 RTL verification.
%
% Generates:
%   1. Impulse
%   2. DC
%   3. Single Tone
%   4. Complex Exponential
%   5. Random Vectors
%
% Saves:
%   vectors/*.txt
%   expected/*.txt
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clearvars;
clear;

config;

% Create folders if they do not exist
if ~exist(cfg.INPUT_FOLDER,'dir')
    mkdir(cfg.INPUT_FOLDER);
end

if ~exist(cfg.OUTPUT_FOLDER,'dir')
    mkdir(cfg.OUTPUT_FOLDER);
end

if ~exist(cfg.REPORT_FOLDER,'dir')
    mkdir(cfg.REPORT_FOLDER);
end

fprintf("=============================================\n");
fprintf("      BS16 Verification Generator\n");
fprintf("=============================================\n\n");

report = {};

%% ================================================================
%% TEST 1 : IMPULSE
%% ================================================================

fprintf("Generating Impulse...\n");

x = zeros(cfg.N,1);
x(1) = 16384;

y = bs16_reference(x);

save_complex_vector(fullfile(cfg.INPUT_FOLDER,...
    "impulse_input.txt"),x);

save_complex_vector(fullfile(cfg.OUTPUT_FOLDER,...
    "impulse_output.txt"),y);

report{end+1} = "Impulse";


%% ================================================================
%% TEST 2 : DC
%% ================================================================

fprintf("Generating DC...\n");

x = 1024*ones(cfg.N,1);

y = bs16_reference(x);

save_complex_vector(fullfile(cfg.INPUT_FOLDER,...
    "dc_input.txt"),x);

save_complex_vector(fullfile(cfg.OUTPUT_FOLDER,...
    "dc_output.txt"),y);

report{end+1} = "DC";


%% ================================================================
%% TEST 3 : SINGLE TONE
%% ================================================================

fprintf("Generating Single Tone...\n");

n = (0:cfg.N-1).';

k = 1;

x = round(1000*cos(2*pi*k*n/cfg.N));

y = bs16_reference(x);

save_complex_vector(fullfile(cfg.INPUT_FOLDER,...
    "single_tone_input.txt"),x);

save_complex_vector(fullfile(cfg.OUTPUT_FOLDER,...
    "single_tone_output.txt"),y);

report{end+1} = "Single Tone";


%% ================================================================
%% TEST 4 : COMPLEX EXPONENTIAL
%% ================================================================

fprintf("Generating Complex Exponential...\n");

n = (0:cfg.N-1).';

k = 1;  %FFT bin

%x = round(2000*exp(1j*2*pi*k*n/cfg.N));
x = round(real(2000*exp(1j*2*pi*k*n/cfg.N))) + ...
    1j*round(imag(2000*exp(1j*2*pi*k*n/cfg.N)));

y = bs16_reference(x);

save_complex_vector(fullfile(cfg.INPUT_FOLDER,...
    "complex_exp_input.txt"),x);

save_complex_vector(fullfile(cfg.OUTPUT_FOLDER,...
    "complex_exp_output.txt"),y);

report{end+1} = "Complex Exponential";


%% ================================================================
%% TEST 5 : RANDOM VECTORS
%% ================================================================

fprintf("Generating Random Vectors...\n");

for t = 1:cfg.NUM_RANDOM

    x = randi([-2000 2000],cfg.N,1) + ...
        1j*randi([-2000 2000],cfg.N,1);

    y = bs16_reference(x);

    input_name = sprintf("random_%02d_input.txt",t);
    output_name = sprintf("random_%02d_output.txt",t);

    save_complex_vector(...
        fullfile(cfg.INPUT_FOLDER,input_name),x);

    save_complex_vector(...
        fullfile(cfg.OUTPUT_FOLDER,output_name),y);

end

report{end+1} = sprintf("%d Random Vectors",cfg.NUM_RANDOM);


%% ================================================================
%% GENERATE REPORT
%% ================================================================

fid = fopen(fullfile(cfg.REPORT_FOLDER,...
    "verification_report.txt"),'w');

if fid == -1
    error("Cannot create verification report.");
end

fprintf(fid,"=================================================\n");
fprintf(fid,"        BS16 VERIFICATION REPORT\n");
fprintf(fid,"=================================================\n\n");

fprintf(fid,"FFT Size        : %d\n",cfg.N);
fprintf(fid,"Word Length     : %d\n",cfg.WORD_LENGTH);
fprintf(fid,"RTL Scaling     : 1/%d\n",cfg.SCALE);
fprintf(fid,"Random Tests    : %d\n\n",cfg.NUM_RANDOM);

fprintf(fid,"Generated Tests\n");
fprintf(fid,"-----------------------------\n");

for k = 1:length(report)

    %fprintf(fid,"%s\n",report{k});
    fprintf(fid,"[PASS] %s Generated\n",report{k});

end

fprintf(fid,"\nGeneration Status : SUCCESS\n");

fprintf(fid,"\n===============================================\n");

fclose(fid);

%% ================================================================
%% SUMMARY
%% ================================================================

fprintf("\n");
fprintf("=============================================\n");
fprintf("Verification vectors generated successfully.\n");
fprintf("=============================================\n\n");

fprintf("Input Folder    : %s\n",cfg.INPUT_FOLDER);
fprintf("Output Folder   : %s\n",cfg.OUTPUT_FOLDER);
fprintf("Report Folder   : %s\n\n",cfg.REPORT_FOLDER);

fprintf("\nGenerated Tests:\n");

for k = 1:length(report)

    fprintf("   %s\n",report{k});

end

fprintf("\nDone.\n");
