clc; clear; close all;

%% تعریف مسیر واقعی (مثل قبل)
t = 0:0.05:10;
dt = t(2)-t(1);
n = length(t);

% مسیر واقعی با دامنه بزرگ‌تر ولی در محدوده تقریبی 0 تا 2
x_true = 1 + 0.9*sin(0.6*t + 0.3);    % بین حدود 0.1 تا 1.9
y_true = 1 + 0.9*cos(0.4*t);          % بین حدود 0.1 تا 1.9
z_true = 1 + 0.8*sin(0.8*t + 1.2);    % بین حدود 0.2 تا 1.8

sensors = [0 0 0;
           2 0 0;
           0 2 0;
           2 2 0];
num_sensors = size(sensors,1);

% مشتق‌ها برای شتاب و سرعت
vx = gradient(x_true, dt);
vy = gradient(y_true, dt);
vz = gradient(z_true, dt);

ax_true = gradient(vx, dt);
ay_true = gradient(vy, dt);
az_true = gradient(vz, dt);

%% مقادیر نویز مختلف
noise_std_acc_list = [0.01, 0.02, 0.05];
noise_std_dist_list = [0.01, 0.03, 0.06];

rms_results = zeros(length(noise_std_acc_list), length(noise_std_dist_list), 4);

for i_acc = 1:length(noise_std_acc_list)
    for j_dist = 1:length(noise_std_dist_list)
        noise_std_acc = noise_std_acc_list(i_acc);
        noise_std_dist = noise_std_dist_list(j_dist);

        % نویز ToF (برد سنسورها)
        dist_true = zeros(num_sensors, n);
        for s = 1:num_sensors
            dist_true(s,:) = sqrt((x_true - sensors(s,1)).^2 + ...
                                 (y_true - sensors(s,2)).^2 + ...
                                 (z_true - sensors(s,3)).^2);
        end
        dist_noisy = dist_true + noise_std_dist * randn(size(dist_true));
        
        pos_est = zeros(3,n);
        options = optimoptions('lsqnonlin', 'Display', 'off');
        for k = 1:n
            d = dist_noisy(:,k);
            fun = @(p) sqrt((p(1) - sensors(:,1)).^2 + ...
                            (p(2) - sensors(:,2)).^2 + ...
                            (p(3) - sensors(:,3)).^2) - d;
            if k == 1
                p0 = [x_true(k), y_true(k), z_true(k)];
            else
                p0 = pos_est(:,k-1)';
            end
            lb = [0 0 0];
            ub = [2 2 2];
            p_est = lsqnonlin(fun, p0, lb, ub, options);
            pos_est(:,k) = p_est;
        end

        x_tof = pos_est(1,:);
        y_tof = pos_est(2,:);
        z_tof = pos_est(3,:);
        err_tof = sqrt(sum((pos_est - [x_true; y_true; z_true]).^2,1));

        % نویز IMU (شتاب)
        ax_noisy = ax_true + noise_std_acc * randn(size(ax_true));
        ay_noisy = ay_true + noise_std_acc * randn(size(ay_true));
        az_noisy = az_true + noise_std_acc * randn(size(az_true));

        vx_est = zeros(1,n); vx_est(1) = vx(1);
        vy_est = zeros(1,n); vy_est(1) = vy(1);
        vz_est = zeros(1,n); vz_est(1) = vz(1);
        x_est = zeros(1,n); x_est(1) = x_true(1);
        y_est = zeros(1,n); y_est(1) = y_true(1);
        z_est = zeros(1,n); z_est(1) = z_true(1);

        for k = 2:n
            vx_est(k) = vx_est(k-1) + ax_noisy(k-1)*dt;
            vy_est(k) = vy_est(k-1) + ay_noisy(k-1)*dt;
            vz_est(k) = vz_est(k-1) + az_noisy(k-1)*dt;

            x_est(k) = x_est(k-1) + vx_est(k-1)*dt;
            y_est(k) = y_est(k-1) + vy_est(k-1)*dt;
            z_est(k) = z_est(k-1) + vz_est(k-1)*dt;
        end
        err_imu = sqrt((x_est - x_true).^2 + (y_est - y_true).^2 + (z_est - z_true).^2);

        % فیلتر کالمن
        A = [1 0 0 dt 0  0;
             0 1 0 0  dt 0;
             0 0 1 0  0  dt;
             0 0 0 1  0  0;
             0 0 0 0  1  0;
             0 0 0 0  0  1];
        B = [0.5*dt^2 0         0;
             0        0.5*dt^2  0;
             0        0         0.5*dt^2;
             dt       0         0;
             0        dt        0;
             0        0         dt];
        Q = (noise_std_acc^2)*eye(6);

        x_comb = zeros(6,n);
        P_comb = eye(6);
        z_meas = [x_tof; y_tof; z_tof];
        H = [1 0 0 0 0 0;
             0 1 0 0 0 0;
             0 0 1 0 0 0];
        R_kf = (noise_std_dist^2) * eye(3);

        x_comb(:,1) = [x_true(1); y_true(1); z_true(1); 0; 0; 0];
        for k = 2:n
            u = [ax_noisy(k-1); ay_noisy(k-1); az_noisy(k-1)];
            x_pred = A*x_comb(:,k-1) + B*u;
            P_pred = A*P_comb*A' + Q;

            z_k = z_meas(:,k);

            y_k = z_k - H*x_pred;
            S_k = H*P_pred*H' + R_kf;
            K_k = P_pred*H'/S_k;

            x_comb(:,k) = x_pred + K_k*y_k;
            P_comb = (eye(6) - K_k*H)*P_pred;
        end
        pos_comb = x_comb(1:3,:);
        err_comb = sqrt(sum((pos_comb - [x_true; y_true; z_true]).^2,1));

        % فیلتر مکمل
        alpha = 0.98;
        pos_filt = zeros(3,n);
        pos_filt(:,1) = [x_true(1); y_true(1); z_true(1)];
        for k=2:n
            delta_imu = [x_est(k) - x_est(k-1);
                         y_est(k) - y_est(k-1);
                         z_est(k) - z_est(k-1)];
            pos_filt(:,k) = alpha * (pos_filt(:,k-1) + delta_imu) + (1-alpha) * [x_tof(k); y_tof(k); z_tof(k)];
        end
        err_comp = sqrt(sum((pos_filt - [x_true; y_true; z_true]).^2,1));

        % ذخیره RMS خطا
        rms_results(i_acc, j_dist, 1) = sqrt(mean(err_tof.^2));
        rms_results(i_acc, j_dist, 2) = sqrt(mean(err_imu.^2));
        rms_results(i_acc, j_dist, 3) = sqrt(mean(err_comb.^2));
        rms_results(i_acc, j_dist, 4) = sqrt(mean(err_comp.^2));

        %% رسم نمودار سه‌بعدی مسیرها
        figure;
        plot3(x_true, y_true, z_true, 'k-', 'LineWidth', 2); hold on;
        plot3(x_tof, y_tof, z_tof, 'r--', 'LineWidth', 1.5);
        plot3(x_est, y_est, z_est, 'b-.', 'LineWidth', 1.5);
        plot3(pos_comb(1,:), pos_comb(2,:), pos_comb(3,:), 'g-', 'LineWidth', 1.5);
        plot3(pos_filt(1,:), pos_filt(2,:), pos_filt(3,:), 'm:', 'LineWidth', 1.5);
        legend('مسیر واقعی', 'ToF', 'IMU', 'کالمن', 'مکمل', 'Location', 'best');
        grid on; box on;
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title(sprintf('مقایسه مسیرها (نویز IMU=%.3f، نویز ToF=%.3f)', noise_std_acc, noise_std_dist));
        axis equal;
        view(45,30);

        %% رسم خطای موقعیت بر حسب زمان
        figure;
        plot(t, err_tof, 'r-', 'LineWidth', 1.5); hold on;
        plot(t, err_imu, 'b-', 'LineWidth', 1.5);
        plot(t, err_comb, 'g-', 'LineWidth', 1.5);
        plot(t, err_comp, 'm-', 'LineWidth', 1.5);
        grid on; box on;
        xlabel('زمان (ثانیه)');
        ylabel('خطای موقعیت (متر)');
        title(sprintf('خطای موقعیت در طول زمان (نویز IMU=%.3f، نویز ToF=%.3f)', noise_std_acc, noise_std_dist));
        legend('خطای ToF', 'خطای IMU', 'خطای کالمن', 'خطای مکمل', 'Location', 'northeast');
    end
end


%% نمایش نتایج RMS کلی
fprintf('نتایج RMS خطا برای ترکیب‌های مختلف نویز IMU و ToF:\n\n');
methods = {'ToF', 'IMU', 'Kalman', 'Complementary'};
for i_acc = 1:length(noise_std_acc_list)
    fprintf('نویز IMU = %.3f\n', noise_std_acc_list(i_acc));
    fprintf('ToF نویز ->  ');
    for j_dist = 1:length(noise_std_dist_list)
        fprintf('%.3f\t', noise_std_dist_list(j_dist));
    end
    fprintf('\n');
    for m = 1:4
        fprintf('%-12s: ', methods{m});
        for j_dist = 1:length(noise_std_dist_list)
            fprintf('%.4f\t', rms_results(i_acc, j_dist, m));
        end
        fprintf('\n');
    end
    fprintf('\n');
end
