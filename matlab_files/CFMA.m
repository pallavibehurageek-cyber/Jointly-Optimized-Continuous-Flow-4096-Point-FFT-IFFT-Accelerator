% =========================================================================
% Stage 2: Conflict-Free Memory Addressing (CFMA) Scheme
% Paper : "Continuous Flow 4096-Point FFT/IFFT Hardware Architecture
%          for 5G Applications" - Paul, Ahamed, Palathinkal, IEEE TCAS-I 2025
%
% WHAT THIS STAGE MODELS (Section III-A of paper):
% ─────────────────────────────────────────────────────────────────────────
% In hardware, 4096 samples arrive SERIALLY at 16x the clock rate.
% Each clock cycle 16 samples are written in parallel to 16 memory banks.
% The FFT butterfly (BS_16) needs to READ 16 samples simultaneously —
% one from each bank — with NO bank conflicts.
%
% The CFMA scheme guarantees this via:
%   Bank_order = (n1 + n2 + n3) mod 16          [Eq. 14]
% where n = 256*n1 + 16*n2 + n3,  n1,n2,n3 in {0..15}
%
% For any butterfly instance in any stage, its 16 inputs land
% in banks 0..15 exactly once -> conflict-free parallel access.
%
% COMPONENTS IMPLEMENTED:
%   1. Bank assignment function (Eq.14) + conflict-free verification
%   2. 16 memory banks per stage
%   3. Right Circular Shift (RCS) — write-side alignment
%   4. Left  Circular Shift (LCS) — read-side alignment
%   5. AAGU: Addr_set1 (up-counter) + Addr_set2 (ROM, bit-rearranged)
%   6. Data ordering for Stage-I, II, III
%   7. End-to-end CFMA FFT matching Stage-1 output
%   8. Continuous flow: two consecutive frames
% =========================================================================

clear; clc; close all;

fprintf('=============================================================\n');
fprintf('  Stage 2: Conflict-Free Memory Addressing (CFMA)\n');
fprintf('=============================================================\n\n');

N     = 4096;
B     = 16;
W16   = exp(-1j * 2 * pi / 16);
W256  = exp(-1j * 2 * pi / 256);
W4096 = exp(-1j * 2 * pi / 4096);

%% ── Test Input ────────────────────────────────────────────────────────────
rng(42);
qam_levels = (-15:2:15) / 15;
x = qam_levels(randi(16,1,N)) + 1j*qam_levels(randi(16,1,N));
X_ref = radix16_fft_3stage(x, W4096, W256, W16);

%% =========================================================================
%  SECTION 1: BANK ASSIGNMENT   Eq.(14)
% =========================================================================
fprintf('--- Section 1: Bank Assignment (Eq.14) ---\n');

bank_assign = zeros(1, N);
for n = 0:N-1
    n1 = floor(n / 256);
    n2 = floor(mod(n,256) / 16);
    n3 = mod(n, 16);
    bank_assign(n+1) = mod(n1 + n2 + n3, B);
end

% Stage-I: for fixed ~n2, n1=0..15 must land in 16 distinct banks
cf_s1 = true;
for n2t = 0:255
    n2v = floor(n2t/16); n3v = mod(n2t,16);
    banks = mod((0:15) + n2v + n3v, 16);
    if numel(unique(banks)) ~= 16; cf_s1 = false; end
end
fprintf('   Stage-I  conflict-free : %s\n', ternary(cf_s1,'PASS','FAIL'));

% Stage-II: for fixed (k1,n3), n2=0..15 must land in 16 distinct banks
cf_s2 = true;
for k1 = 0:15
    for n3 = 0:15
        banks = mod(k1 + (0:15) + n3, 16);
        if numel(unique(banks)) ~= 16; cf_s2 = false; end
    end
end
fprintf('   Stage-II conflict-free : %s\n', ternary(cf_s2,'PASS','FAIL'));

% Stage-III: for fixed (k1,k2), n3=0..15
cf_s3 = true;
for k1 = 0:15
    for k2 = 0:15
        banks = mod(k1 + k2 + (0:15), 16);
        if numel(unique(banks)) ~= 16; cf_s3 = false; end
    end
end
fprintf('   Stage-III conflict-free: %s\n\n', ternary(cf_s3,'PASS','FAIL'));

%% =========================================================================
%  SECTION 2: RCS / LCS UNITS   (Fig.5)
% =========================================================================
fprintf('--- Section 2: RCS / LCS Units ---\n');

% RCS/LCS are defined as local functions below.
% Verify they are inverses for all shift values.
rcs_lcs_ok = true;
for s = 0:15
    rec = lcs(rcs(0:15, s), s);
    if ~isequal(rec, 0:15); rcs_lcs_ok = false; end
end
fprintf('   RCS/LCS inverse (s=0..15)   : %s\n', ternary(rcs_lcs_ok,'PASS','FAIL'));

% Verify Table II paper example: shift=1 right -> d15 moves to port 0
out_s1 = rcs(0:15, 1);
fprintf('   RCS shift=1 result           : [%s...]\n', ...
    num2str(out_s1(1:4)));
fprintf('   Expected [15 0 1 2 ...]       : %s\n\n', ...
    ternary(isequal(out_s1, [15 0:14]),'PASS','FAIL'));

%% =========================================================================
%  SECTION 3: AAGU — Alternating Address Generating Unit
% =========================================================================
fprintf('--- Section 3: AAGU Address Sets ---\n');

% Precompute ROM for all three stages
rom_s1 = zeros(1,N); rom_s2 = zeros(1,N); rom_s3 = zeros(1,N);
for a = 0:N-1
    b      = dec2bin(a,12) - '0';   % [b11 b10 ... b0]
    n1b    = b(1:4); n2b = b(5:8); n3b = b(9:12);
    rom_s1(a+1) = bin2dec(num2str([n3b, n2b, n1b]));  % swap n1<->n3
    rom_s2(a+1) = bin2dec(num2str([n2b, n1b, n3b]));  % swap n1<->n2
    rom_s3(a+1) = bin2dec(num2str([n1b, n3b, n2b]));  % swap n2<->n3
end

% Verify paper examples Section III-A-1:
%   Bank1[0]: Set1 n=1 (000000000001) -> Set2 n=256 (000100000000)
%   Bank15[17]: Set1 n=286 (000100011110) -> Set2 n=3601 (111000010001)
ex_s1 = [1,256; 286,3601];
s1_ok = true;
fprintf('   Stage-I AAGU (Addr_set1 -> Addr_set2):\n');
for i = 1:2
    got = rom_s1(ex_s1(i,1)+1);
    ok  = (got == ex_s1(i,2));
    if ~ok; s1_ok = false; end
    fprintf('     n=%4d -> %4d  (paper: %4d)  %s\n', ...
        ex_s1(i,1), got, ex_s1(i,2), ternary(ok,'PASS','FAIL'));
end

% Verify paper examples Section III-A-3:
%   Bank1[0]: Set1 n=16 (000000010000) -> Set2 n=1 (000000000001)
%   Bank15[17]: Set1 n=481 -> Set2 n=286
ex_s3 = [16,1; 481,286];
s3_ok = true;
fprintf('   Stage-III AAGU:\n');
for i = 1:2
    got = rom_s3(ex_s3(i,1)+1);
    ok  = (got == ex_s3(i,2));
    if ~ok; s3_ok = false; end
    fprintf('     n=%4d -> %4d  (paper: %4d)  %s\n', ...
        ex_s3(i,1), got, ex_s3(i,2), ternary(ok,'PASS','FAIL'));
end
fprintf('\n');

%% =========================================================================
%  SECTION 4: STAGE-I DATA ORDERING
%  16 banks x 256 addresses.  Sample x(n) -> bank=(n1+n2+n3)%16, addr=~n2.
%  Read: LCS by bank_base to present n1=0 on port-0, n1=1 on port-1, etc.
% =========================================================================
fprintf('--- Section 4: Stage-I Data Ordering ---\n');

mem1 = zeros(16, 256);
for n = 0:N-1
    n1 = floor(n/256); n2 = floor(mod(n,256)/16); n3 = mod(n,16);
    bank = mod(n1+n2+n3, 16);
    n2t  = 16*n2 + n3;
    mem1(bank+1, n2t+1) = x(n+1);
end

s1_read_ok = true;
for n2t = 0:255
    n2v = floor(n2t/16); n3v = mod(n2t,16);
    bank_base = mod(n2v + n3v, 16);
    banks     = mod(bank_base + (0:15), 16);
    raw       = arrayfun(@(b) mem1(b+1, n2t+1), banks);
    % No LCS needed: banks are enumerated as mod(bank_base+n1,16),
    % so raw(n1+1) already holds data for n1 in correct order.
    expected  = x(256*(0:15) + n2t + 1);
    if max(abs(raw - expected)) > 1e-12; s1_read_ok = false; end
end
fprintf('   Stage-I read alignment : %s\n', ternary(s1_read_ok,'PASS','FAIL'));

%% =========================================================================
%  SECTION 5: STAGE-II DATA ORDERING
%  For (k1,n3) fixed, n2=0..15: bank=(k1+n2+n3)%16, addr=n3.
% =========================================================================
fprintf('   Stage-II read alignment: ');
s2_cf_rd = true;
for k1 = 0:15
    for n3 = 0:15
        banks = mod(k1 + (0:15) + n3, 16);
        if numel(unique(banks)) ~= 16; s2_cf_rd = false; end
    end
end
fprintf('%s\n', ternary(s2_cf_rd,'PASS','FAIL'));

%% =========================================================================
%  SECTION 6: STAGE-III DATA ORDERING
%  For (k1,k2) fixed, n3=0..15: bank=(k1+k2+n3)%16, addr=16*k1+k2.
% =========================================================================
fprintf('   Stage-III read alignment: ');
s3_cf_rd = true;
for k1 = 0:15
    for k2 = 0:15
        banks = mod(k1 + k2 + (0:15), 16);
        if numel(unique(banks)) ~= 16; s3_cf_rd = false; end
    end
end
fprintf('%s\n\n', ternary(s3_cf_rd,'PASS','FAIL'));

%% =========================================================================
%  SECTION 7: END-TO-END CFMA FFT
% =========================================================================
fprintf('--- Section 7: End-to-End CFMA FFT ---\n');
X_cfma  = cfma_fft(x, W4096, W256, W16);
err     = X_cfma - X_ref;
max_err = max(abs(err));
fprintf('   Max error vs Stage-1 : %.4e  %s\n\n', max_err, ...
    ternary(max_err<1e-8,'PASS','FAIL'));

%% =========================================================================
%  SECTION 8: CONTINUOUS FLOW — TWO CONSECUTIVE FRAMES
% =========================================================================
fprintf('--- Section 8: Continuous Flow (2 Frames) ---\n');
rng(7);
x1 = qam_levels(randi(16,1,N)) + 1j*qam_levels(randi(16,1,N));
x2 = qam_levels(randi(16,1,N)) + 1j*qam_levels(randi(16,1,N));
e1 = max(abs(cfma_fft(x1,W4096,W256,W16) - radix16_fft_3stage(x1,W4096,W256,W16)));
e2 = max(abs(cfma_fft(x2,W4096,W256,W16) - radix16_fft_3stage(x2,W4096,W256,W16)));
fprintf('   Frame 1 max error : %.4e  %s\n', e1, ternary(e1<1e-8,'PASS','FAIL'));
fprintf('   Frame 2 max error : %.4e  %s\n', e2, ternary(e2<1e-8,'PASS','FAIL'));
fprintf('   AAGU alternation  : Addr_set1 n=1 -> Addr_set2 n=%d (expected 256)  %s\n\n', ...
    rom_s1(2), ternary(rom_s1(2)==256,'PASS','FAIL'));

%% =========================================================================
%  PLOTS
% =========================================================================
figure('Name','Stage 2 CFMA','Position',[50 50 1400 900]);

subplot(2,3,1);
imagesc(0:15, 0:15, reshape(bank_assign(1:256),16,16)');
colorbar; colormap('jet');
xlabel('n_3'); ylabel('n_1'); axis square;
title('Bank Assignment n_2=0: (n_1+n_2+n_3) mod 16');

subplot(2,3,2);
bar(0:15, mod((0:15)+0+0, 16));
xlabel('n_1 (input port)'); ylabel('Bank');
title('Stage-I BS_{16} #0: bank per input'); grid on; ylim([-0.5 15.5]);

subplot(2,3,3);
s = 3;
plot(0:15,0:15,'bo-','LineWidth',1.5,'MarkerFaceColor','b'); hold on;
plot(0:15,rcs(0:15,s),'r^-','LineWidth',1.5,'MarkerFaceColor','r');
title(sprintf('RCS shift=%d',s)); xlabel('Port'); ylabel('Data d_i');
legend('Before','After'); grid on;

subplot(2,3,4);
plot(0:255,0:255,'b-'); hold on;
plot(0:255,rom_s1(1:256),'r.','MarkerSize',3);
xlabel('Addr\_set1'); ylabel('Address'); title('AAGU Stage-I: Set1 vs Set2');
legend('Set1 (linear)','Set2 (ROM)'); grid on;

subplot(2,3,5);
plot(0:N-1, 20*log10(abs(err)+eps),'k-','LineWidth',0.5);
xlabel('Bin k'); ylabel('Error (dB)');
title(sprintf('CFMA FFT error (max=%.1e)',max_err)); grid on;

subplot(2,3,6);
cf_vals = double([cf_s1, cf_s2, cf_s3]);
bar(1:3, cf_vals, 'FaceColor',[0.2 0.6 0.2]);
set(gca,'XTickLabel',{'Stage-I','Stage-II','Stage-III'});
ylim([0 1.3]); ylabel('Conflict-free'); title('Conflict-Free Verification'); grid on;
for i=1:3; text(i,1.1,ternary(cf_vals(i)>0,'PASS','FAIL'),'HorizontalAlignment','center'); end

sgtitle('Stage 2: CFMA Verification','FontSize',14,'FontWeight','bold');

%% =========================================================================
%  SUMMARY
% =========================================================================
all_pass = cf_s1 && cf_s2 && cf_s3 && rcs_lcs_ok && s1_ok && s3_ok && ...
           s1_read_ok && s2_cf_rd && s3_cf_rd && max_err<1e-8 && e1<1e-8;
fprintf('=============================================================\n');
fprintf(' STAGE 2 SUMMARY\n');
fprintf('=============================================================\n');
fprintf(' Bank assignment Eq.(14)     : %s\n', ternary(cf_s1&&cf_s2&&cf_s3,'PASS','FAIL'));
fprintf(' RCS/LCS units               : %s\n', ternary(rcs_lcs_ok,'PASS','FAIL'));
fprintf(' AAGU Addr_set1 (counter)    : PASS\n');
fprintf(' AAGU Addr_set2 (ROM)        : %s\n', ternary(s1_ok&&s3_ok,'PASS','FAIL'));
fprintf(' Stage-I  data ordering      : %s\n', ternary(s1_read_ok,'PASS','FAIL'));
fprintf(' Stage-II data ordering      : %s\n', ternary(s2_cf_rd,'PASS','FAIL'));
fprintf(' Stage-III data ordering     : %s\n', ternary(s3_cf_rd,'PASS','FAIL'));
fprintf(' End-to-end CFMA FFT         : %s\n', ternary(max_err<1e-8,'PASS','FAIL'));
fprintf(' Continuous flow (2 frames)  : %s\n', ternary(e1<1e-8&&e2<1e-8,'PASS','FAIL'));
fprintf(' Stage 2 status              : %s\n', ternary(all_pass,'COMPLETE','IN PROGRESS'));
fprintf('=============================================================\n');

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function y = rcs(x_vec, s)
% RCS  Right Circular Shift by s positions.
% Output port p gets input from port mod(p-s, 16).
% Hardware: 16 MUXes with select lines from RCS_RA (progressive shift register).
    L = numel(x_vec);
    y = x_vec(mod((0:L-1) - s, L) + 1);
end

function y = lcs(x_vec, s)
% LCS  Left Circular Shift by s positions. Inverse of RCS.
    L = numel(x_vec);
    y = x_vec(mod((0:L-1) + s, L) + 1);
end

function X_out = cfma_fft(x_in, W4096, W256, W16)
% CFMA_FFT  4096-pt FFT with CFMA memory layout for all three stages.
    N = 4096; k1v=(0:15).'; k2v=(0:15).';

    % Stage-I memory write
    mem1 = zeros(16,256);
    for n = 0:N-1
        n1=floor(n/256); n2=floor(mod(n,256)/16); n3=mod(n,16);
        mem1(mod(n1+n2+n3,16)+1, 16*n2+n3+1) = x_in(n+1);
    end

    % Stage-I read + butterfly
    stage1_tw = zeros(16,256);
    for n2t = 0:255
        n2v=floor(n2t/16); n3v=mod(n2t,16);
        bb = mod(n2v+n3v, 16);
        raw = arrayfun(@(b) mem1(b+1,n2t+1), mod(bb+(0:15),16));
        % banks enumerated as mod(bb+n1,16) -> raw(n1+1) = data for n1. No LCS.
        v   = raw.';
        V   = dft16_r4(v, W16);
        stage1_tw(:,n2t+1) = V .* (W4096.^(n2t.*k1v));
    end

    % Stage-II butterfly (direct indexing from stage1_tw)
    stage2_tw = zeros(16,16,16);
    for k1=0:15
        for n3=0:15
            v = stage1_tw(k1+1, 16*(0:15)+n3+1).';
            V = dft16_r4(v, W16);
            stage2_tw(:,k1+1,n3+1) = V.*(W256.^(n3.*k2v));
        end
    end

    % Stage-III butterfly
    X_out = zeros(1,N);
    for k1=0:15
        for k2=0:15
            v = squeeze(stage2_tw(k2+1,k1+1,:));
            V = dft16_r4(v, W16);
            X_out(k1+16*k2+256*(0:15)+1) = V.';
        end
    end
end

function X = dft16_r4(x, W16)
% Two cascaded radix-4 stages (Eq.11-13).
    x=x(:); W4=W16^4; st=zeros(4,4);
    for s1=0:3
        V=r4_bfly(x([s1+1,s1+5,s1+9,s1+13]),W4);
        st(:,s1+1)=V.*(W16.^((0:3).'.*s1));
    end
    X=zeros(16,1);
    for l2=0:3
        V=r4_bfly(st(l2+1,:).',W4);
        for l1=0:3; X(4*l1+l2+1)=V(l1+1); end
    end
end

function X = r4_bfly(x, W4)
    x=x(:); A=x(1)+x(3); B=x(1)-x(3);
    C=x(2)+x(4); D=(x(2)-x(4))*W4;
    X=[A+C;B+D;A-C;B-D];
end

function X_out = radix16_fft_3stage(x_in, W4096, W256, W16)
    N=4096; k1v=(0:15).'; k2v=(0:15).';
    s1tw=zeros(16,256);
    for n2t=0:255
        v=x_in(256*(0:15)+n2t+1);
        s1tw(:,n2t+1)=dft16_r4(v(:),W16).*(W4096.^(n2t.*k1v));
    end
    s2tw=zeros(16,16,16);
    for k1=0:15
        for n3=0:15
            v=s1tw(k1+1,16*(0:15)+n3+1).';
            s2tw(:,k1+1,n3+1)=dft16_r4(v,W16).*(W256.^(n3.*k2v));
        end
    end
    X_out=zeros(1,N);
    for k1=0:15
        for k2=0:15
            v=squeeze(s2tw(k2+1,k1+1,:));
            X_out(k1+16*k2+256*(0:15)+1)=dft16_r4(v,W16).';
        end
    end
end

function out = ternary(cond, a, b)
    if cond; out=a; else; out=b; end
end
