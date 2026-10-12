% =========================================================================
% Stage 1: 4096-Point Radix-16 FFT/IFFT
% Paper : "Continuous Flow 4096-Point FFT/IFFT Hardware Architecture
%          for 5G Applications" - Paul, Ahamed, Palathinkal, IEEE TCAS-I 2025
%
% ALGORITHM  (Equations 4-13 of paper)
% ─────────────────────────────────────────────────────────────────────────
% Three outer stages, each performing 256 × BS_16 operations:
%
%   Index decomposition:
%     n = 256·n1 + 16·n2 + n3,   n1,n2,n3 ∈ {0..15}
%     k = k1 + 16·k2 + 256·k3,  k1,k2,k3 ∈ {0..15}
%
%   Outer Stage I  (Eq.6):
%     For each ~n2 ∈ {0..255}: BS_16 over n1, twiddle W4096^(~n2·k1)
%
%   Outer Stage II (Eq.8):
%     For each (k1,n3): BS_16 over n2, twiddle W256^(n3·k2)
%
%   Outer Stage III (Eq.9-10):
%     For each (k1,k2): BS_16 over n3, no twiddle
%     Output bin: k = k1 + 16·k2 + 256·k3
%
% BS_16 UNIT  (Equations 11-13 of paper)
% ─────────────────────────────────────────────────────────────────────────
% Each BS_16 is built from two cascaded radix-4 stages:
%   n1 = 4·s2 + s1,  k1 = 4·l1 + l2,  s1,s2,l1,l2 ∈ {0..3}
%
%   BS_16 Stage-I  (Eq.11): 4× BS_4 (stride-4 decimation) + twiddle W16^(s1·l2)
%   BS_16 Stage-II (Eq.12): 4× BS_4, output → bin k1 = 4·l1 + l2  (Eq.13)
%
% IFFT  (Equation 2 of paper)
% ─────────────────────────────────────────────────────────────────────────
%   IFFT(X)[n] = (1/N) · conj( FFT( conj(X) ) )[n]
%   Same hardware, conjugation applied at input and output.
% =========================================================================

clear; clc; close all;

%% -------------------------------------------------------------------------
%  PARAMETERS
% -------------------------------------------------------------------------
N  = 4096;
r  = 16;
fprintf('=============================================================\n');
fprintf(' 4096-Point Radix-16 FFT  (Three Cascaded 16-Point DFTs)\n');
fprintf('=============================================================\n');
fprintf('N = %d,  Radix = %d,  Stages = 3\n\n', N, r);

%% -------------------------------------------------------------------------
%  TWIDDLE FACTOR ROOTS
% -------------------------------------------------------------------------
W4096 = exp(-1j * 2 * pi / 4096);   % W_{4096}
W256  = exp(-1j * 2 * pi / 256);    % W_{256}  = W_{4096}^{16}
W16   = exp(-1j * 2 * pi / 16);     % W_{16}   = W_{4096}^{256}

%% =========================================================================
%  TEST SUITE
%  Run multiple test cases to validate thoroughly:
%    TC1 - Impulse at n=0        (FFT should be all-ones)
%    TC2 - Single complex tone   (FFT should be a single spike)
%    TC3 - All-ones              (FFT spike at k=0 only)
%    TC4 - 256-QAM random frame  (paper's actual test case, seed=42)
%    TC5 - 256-QAM random frame  (different seed for generality)
% =========================================================================
test_cases = struct();

% TC1: Impulse at n=0
x_imp        = zeros(1,N); x_imp(1) = 1;
test_cases(1).name = 'Impulse at n=0';
test_cases(1).x    = x_imp;

% TC2: Pure tone at frequency bin k=100
test_cases(2).name = 'Pure tone (k=100)';
test_cases(2).x    = exp(1j*2*pi*100*(0:N-1)/N);

% TC3: All ones
test_cases(3).name = 'All-ones';
test_cases(3).x    = ones(1,N);

% TC4: 256-QAM random (paper seed)
rng(42);
qam_levels   = (-15:2:15) / 15;
test_cases(4).name = '256-QAM (seed=42, paper test)';
test_cases(4).x    = qam_levels(randi(16,1,N)) + ...
                     1j*qam_levels(randi(16,1,N));

% TC5: 256-QAM random (different seed)
rng(123);
test_cases(5).name = '256-QAM (seed=123)';
test_cases(5).x    = qam_levels(randi(16,1,N)) + ...
                     1j*qam_levels(randi(16,1,N));

num_tc = numel(test_cases);
all_pass = true;

fprintf('%-30s  %12s  %12s  %10s  %s\n', ...
    'Test Case','Max Error','RMS Error','SQNR(dB)','Status');
fprintf('%s\n', repmat('-',1,80));

for tc = 1:num_tc
    x   = test_cases(tc).x;
    
    %% --- Reference ---
    X_ref = fft(x);
    
    %% -----------------------------------------------------------------
    %  STAGE I
    %  For each ~n2 in {0..255}:
    %    gather x(256*n1 + ~n2) for n1=0..15
    %    16-pt DFT over n1  ->  BS^{1,~n2}_{16}(k1)
    %    multiply by W4096^(~n2 * k1)
    %  Result: stage1_tw(k1+1, ~n2+1),  size [16 x 256]
    % -----------------------------------------------------------------
    stage1_tw = zeros(16, 256);
    k1vec     = (0:15).';                     % column vector

    for n2t = 0:255                           % ~n2 = 16*n2 + n3
        v = x(256*(0:15) + n2t + 1);         % length-16 input
        V = dft16_direct(v, W16);             % 16-pt DFT
        stage1_tw(:, n2t+1) = V .* (W4096 .^ (n2t .* k1vec));
    end

    %% -----------------------------------------------------------------
    %  STAGE II
    %  stage1_tw is [16 x 256] with cols indexed by ~n2 = 16*n2 + n3.
    %  We need to iterate over (k1, n3) and gather n2=0..15:
    %    for fixed k1, n3:  input(n2) = stage1_tw(k1+1, 16*n2+n3+1)
    %
    %  Direct indexing (no reshape ambiguity):
    %    col index for (n2, n3) = 16*n2 + n3
    %  Result: stage2_tw(k2+1, k1+1, n3+1),  size [16 x 16 x 16]
    % -----------------------------------------------------------------
    stage2_tw = zeros(16, 16, 16);            % (k2, k1, n3)
    k2vec     = (0:15).';

    for k1 = 0:15
        for n3 = 0:15
            % Gather stage-I outputs for n2=0..15, fixed k1 and n3
            col_indices = 16*(0:15) + n3 + 1; % cols: ~n2=16*n2+n3, 1-based
            v = stage1_tw(k1+1, col_indices);  % row slice -> length-16

            % 16-pt DFT over n2
            V = dft16_direct(v(:), W16);

            % Twiddle: W256^(n3 * k2)
            stage2_tw(:, k1+1, n3+1) = V .* (W256 .^ (n3 .* k2vec));
        end
    end

    %% -----------------------------------------------------------------
    %  STAGE III
    %  For each (k1, k2) pair, gather stage2_tw over n3=0..15:
    %    16-pt DFT over n3  ->  no twiddle after this stage
    %  Assemble output: k = k1 + 16*k2 + 256*k3
    % -----------------------------------------------------------------
    X_out = zeros(1, N);

    for k1 = 0:15
        for k2 = 0:15
            % Gather stage-II outputs for n3=0..15, fixed k1 and k2
            v = squeeze(stage2_tw(k2+1, k1+1, :));  % length-16

            % 16-pt DFT over n3
            V = dft16_direct(v, W16);

            % Place into output bins: k = k1 + 16*k2 + 256*k3
            k_bins = k1 + 16*k2 + 256*(0:15) + 1;   % 1-based
            X_out(k_bins) = V.';
        end
    end

    %% --- Metrics ---
    err         = X_out - X_ref;
    max_err     = max(abs(err));
    rms_err     = sqrt(mean(abs(err).^2));
    sig_pwr     = mean(abs(X_ref).^2);
    noise_pwr   = mean(abs(err).^2);
    if noise_pwr > 0
        SQNR = 10*log10(sig_pwr / noise_pwr);
    else
        SQNR = Inf;
    end
    pass = (max_err < 1e-6);
    all_pass = all_pass && pass;

    fprintf('%-30s  %12.4e  %12.4e  %10.2f  %s\n', ...
        test_cases(tc).name, max_err, rms_err, SQNR, ...
        ternary(pass,'PASS','FAIL'));

    % Save TC4 (paper test case) for detailed plots
    if tc == 4
        X_radix16_paper = X_out;
        X_ref_paper     = X_ref;
        x_paper         = x;
        SQNR_paper      = SQNR;
        err_paper       = err;
    end
end

fprintf('%s\n', repmat('-',1,80));
fprintf('Overall result: %s\n\n', ternary(all_pass,'ALL PASS','SOME FAILED'));

%% =========================================================================
%  IFFT VERIFICATION  (Equation 2 of paper)
%  IFFT(x) = conj( FFT( conj(x) ) ) / N
%  Tested on TC4 (256-QAM, paper test case)
% =========================================================================
fprintf('=============================================================\n');
fprintf(' IFFT VERIFICATION  (Conjugation Trick - Eq.2 of paper)\n');
fprintf('=============================================================\n');

% Paper Eq.(2): IFFT(X)[n] = (1/N) * conj( FFT( conj(X) ) )[n]
% Hardware flow:
%   Step 1 - Conjugate the FREQUENCY-domain input X before feeding to FFT core
%   Step 2 - Pass conj(X) through the same FFT hardware
%   Step 3 - Conjugate the output and divide by N
%
% To test this end-to-end (time -> freq -> time):
%   X_freq      = FFT(x_paper)            using our radix-16 core
%   x_recov     = IFFT(X_freq)
%               = conj( FFT( conj(X_freq) ) ) / N

% Step A: forward FFT of time-domain signal
X_freq      = radix16_fft_core(x_paper, W4096, W256, W16);

% Verify forward FFT matches reference (sanity check)
fft_check_err = max(abs(X_freq - X_ref_paper));
fprintf('Forward FFT check (should be ~0) : %.4e\n', fft_check_err);

% Step B: IFFT via conjugation trick (Eq.2 of paper)
%   Feed conj(X_freq) into FFT hardware, then conjugate output and scale
X_freq_conj = conj(X_freq);                                % conjugate freq input
X_conj_fft  = radix16_fft_core(X_freq_conj, W4096, W256, W16); % FFT hardware
x_recov     = conj(X_conj_fft) / N;                        % conjugate + scale

ifft_err     = x_recov - x_paper;
max_ifft_err = max(abs(ifft_err));
fprintf('Input : 256-QAM frame (seed=42), N=%d\n', N);
fprintf('Max IFFT reconstruction error : %.4e\n', max_ifft_err);
fprintf('Status: %s\n\n', ternary(max_ifft_err < 1e-6,'PASS','FAIL'));

%% =========================================================================
%  PLOTS  (using TC4 - paper test case)
% =========================================================================
k_ax = 0:N-1;

figure('Name','Radix-16 FFT Verification','Position',[50 50 1300 950]);

subplot(3,2,1);
plot(k_ax, real(X_ref_paper), 'b-', 'LineWidth',0.8); hold on;
plot(k_ax, real(X_radix16_paper), 'r--', 'LineWidth',0.8);
title('FFT Real Part'); xlabel('Bin k'); ylabel('Amplitude');
legend('MATLAB fft()','Radix-16 (3-stage)'); grid on;

subplot(3,2,2);
plot(k_ax, imag(X_ref_paper), 'b-', 'LineWidth',0.8); hold on;
plot(k_ax, imag(X_radix16_paper), 'r--', 'LineWidth',0.8);
title('FFT Imaginary Part'); xlabel('Bin k'); ylabel('Amplitude');
legend('MATLAB fft()','Radix-16 (3-stage)'); grid on;

subplot(3,2,3);
plot(k_ax, abs(err_paper), 'k-', 'LineWidth',0.6);
title(sprintf('Absolute Error per Bin  (Max=%.2e)', max(abs(err_paper))));
xlabel('Bin k'); ylabel('|Error|'); grid on;

subplot(3,2,4);
plot(k_ax, 20*log10(abs(X_ref_paper)+eps), 'b-', 'LineWidth',0.8); hold on;
plot(k_ax, 20*log10(abs(X_radix16_paper)+eps), 'r--', 'LineWidth',0.8);
title('Magnitude Spectrum (dB)'); xlabel('Bin k'); ylabel('dB');
legend('Reference','Radix-16'); grid on;

subplot(3,2,5);
plot(0:99, real(x_paper(1:100)), 'b-', 'LineWidth',0.8); hold on;
plot(0:99, real(x_recov(1:100)), 'r--', 'LineWidth',0.8);
title('IFFT Reconstruction - Real Part (first 100 samples)');
xlabel('n'); ylabel('Amplitude');
legend('Original x[n]','Recovered'); grid on;

subplot(3,2,6);
scatter(real(x_paper), imag(x_paper), 4, 'b', 'o'); hold on;
scatter(real(x_recov), imag(x_recov), 4, 'r', 'filled');
title('256-QAM Constellation: Original vs Recovered');
xlabel('In-Phase'); ylabel('Quadrature');
legend('Original','Recovered (IFFT)'); grid on;

sgtitle('Stage 1: 4096-Point Radix-16 FFT/IFFT Verification (v2 - Fixed)', ...
    'FontSize',13,'FontWeight','bold');

%% =========================================================================
%  COMPLEXITY SUMMARY
% =========================================================================
fprintf('=============================================================\n');
fprintf(' COMPLEXITY ANALYSIS\n');
fprintf('=============================================================\n');
fprintf('Algorithm             : Radix-16, N=%d\n', N);
fprintf('DFT    O(N^2)         : %d complex mults\n', N^2);
fprintf('FFT    O(N log2 N)    : %d complex mults\n', round(N*log2(N)));
fprintf('Radix-16 O(N log16 N) : %d complex mults\n', round(N*log(N)/log(16)));
fprintf('\n--- BS_16 unit (built from BS_4, Eq.11-13) ---\n');
fprintf('  Stage-I  : 4x radix-4 butterflies (inputs: stride-4 decimation)\n');
fprintf('             + 12 intra-twiddle mults W16^(s1*l2)\n');
fprintf('  Stage-II : 4x radix-4 butterflies\n');
fprintf('BS16 consists of 8 radix-4 butterflies and 12 non-trivial twiddle factors.\n');
fprintf('\n--- Full 4096-pt FFT (3 outer stages) ---\n');
fprintf('  BS_16 calls          : %d  (256 per outer stage x 3)\n', 3*256);
fprintf('  Inter-stage twiddles : %d  (256 x 16 x 2 stages)\n', 2*256*16);
fprintf('=============================================================\n');
fprintf(' FINAL SUMMARY\n');
fprintf('=============================================================\n');
fprintf('Algorithm : Radix-16, n=256n1+16n2+n3, k=k1+16k2+256k3\n');
fprintf('SQNR (TC4): %.2f dB  (paper target: 51.2 dB in fixed-point HW)\n', SQNR_paper);
fprintf('Overall   : %s\n', ternary(all_pass,'ALL TEST CASES PASS','SOME FAILED'));
fprintf('=============================================================\n');

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function X = dft16_direct(x, W16)
% DFT16_DIRECT  16-point DFT implemented via TWO CASCADED RADIX-4 STAGES
%               exactly as described in Equations (11)-(13) of the paper.
%
% Hardware mapping (BS_16 built from BS_4):
%   Decompose index n1 = 4*s2 + s1,  k1 = 4*l1 + l2,  s1,s2,l1,l2 in {0..3}
%
%   STAGE-I  (Eq.11): 4 parallel radix-4 butterflies, one per s1
%     s1=0: inputs x(0), x(4), x(8),  x(12)
%     s1=1: inputs x(1), x(5), x(9),  x(13)
%     s1=2: inputs x(2), x(6), x(10), x(14)
%     s1=3: inputs x(3), x(7), x(11), x(15)
%     After each 4-pt DFT, multiply output l2 by intra-BS16 twiddle W16^(s1*l2)
%
%   STAGE-II (Eq.12): 4 parallel radix-4 butterflies, one per l2
%     l2-th butterfly input: stage-I twiddle-multiplied output [s1=0..3, l2]
%     Output l1 of l2-th butterfly -> final bin k1 = 4*l1 + l2
%
% Complexity: 8x radix-4 DFTs + 12 twiddle mults = 60 complex mults
%             vs 256 for direct DFT (76.6% saving, matching paper claim)
%
% Input  x : 16x1 or 1x16 complex vector
% Output X : 16x1 complex vector, X(k+1) = sum_{n=0}^{15} x(n)*W16^(nk)

    x = x(:);                          % ensure column vector
    W4 = W16^4;                        % W_4 = exp(-j2pi/4) = W_16^4

    %% --- STAGE I: 4 radix-4 butterflies (over s2, for each s1) ----------
    % stage1_tw(l2+1, s1+1) = BS4 output l2 for input-group s1,
    %                          after twiddle W16^(s1*l2)
    stage1_tw = zeros(4, 4);           % (l2, s1)
    for s1 = 0:3
        % Gather inputs: x(4*s2 + s1) for s2 = 0,1,2,3
        v = x([s1+1, s1+5, s1+9, s1+13]);    % 4 inputs for this butterfly
        V = radix4_butterfly(v, W4);           % 4-pt DFT -> V(l2), l2=0..3
        % Apply intra-BS16 twiddle: W16^(s1 * l2)
        l2vec = (0:3).';
        stage1_tw(:, s1+1) = V .* (W16 .^ (s1 .* l2vec));
    end

    %% --- STAGE II: 4 radix-4 butterflies (over s1, for each l2) ---------
    X = zeros(16, 1);
    for l2 = 0:3
        % Gather: stage1_tw(l2, s1) for s1=0,1,2,3
        v = stage1_tw(l2+1, :).';            % 4x1 column vector
        V = radix4_butterfly(v, W4);          % 4-pt DFT -> V(l1), l1=0..3
        % Place into output: k1 = 4*l1 + l2
        for l1 = 0:3
            X(4*l1 + l2 + 1) = V(l1+1);
        end
    end
end

function X = radix4_butterfly(x, W4)
% RADIX4_BUTTERFLY  4-point DFT butterfly
% X(k) = sum_{n=0}^{3} x(n) * W4^(nk),  W4 = exp(-j2pi/4) = -j
% In hardware implemented with 3 stages of add/subtract (no multiplications
% except by +/-1 and +/-j, which are trivial rotations).
%
% Signal flow:
%   A = x(0)+x(2),  B = x(0)-x(2)
%   C = x(1)+x(3),  D = (x(1)-x(3))*(-j)
%   X(0)=A+C, X(1)=B+D, X(2)=A-C, X(3)=B-D
    x = x(:);
    A = x(1) + x(3);          % x(0) + x(2)
    B = x(1) - x(3);          % x(0) - x(2)
    C = x(2) + x(4);          % x(1) + x(3)
    D = (x(2) - x(4)) * W4;   % (x(1) - x(3)) * W4  = (x(1)-x(3))*(-j)
    X = [A+C; B+D; A-C; B-D];
end

function X_out = radix16_fft_core(x_in, W4096, W256, W16)
% RADIX16_FFT_CORE  Reusable 3-stage radix-16 FFT kernel.
% Identical algorithm to main body; used for IFFT conjugation test.
    N     = 4096;
    k1vec = (0:15).';
    k2vec = (0:15).';
    X_out = zeros(1, N);

    % Stage I
    stage1_tw = zeros(16, 256);
    for n2t = 0:255
        v = x_in(256*(0:15) + n2t + 1);
        V = dft16_direct(v(:), W16);
        stage1_tw(:, n2t+1) = V .* (W4096 .^ (n2t .* k1vec));
    end

    % Stage II
    stage2_tw = zeros(16, 16, 16);
    for k1 = 0:15
        for n3 = 0:15
            col_idx = 16*(0:15) + n3 + 1;
            v = stage1_tw(k1+1, col_idx);
            V = dft16_direct(v(:), W16);
            stage2_tw(:, k1+1, n3+1) = V .* (W256 .^ (n3 .* k2vec));
        end
    end

    % Stage III
    for k1 = 0:15
        for k2 = 0:15
            v = squeeze(stage2_tw(k2+1, k1+1, :));
            V = dft16_direct(v, W16);
            k_bins = k1 + 16*k2 + 256*(0:15) + 1;
            X_out(k_bins) = V.';
        end
    end
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end
