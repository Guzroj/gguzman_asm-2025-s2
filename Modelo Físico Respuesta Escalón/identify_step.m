% identify_step.m
% Identifica modelos de primer orden para ambos movimientos (alejamiento y acercamiento)

clc; clear; close all;

pkg load optim
pkg load control

% --- parámetros usuario ---
csvfile = 'datos_puerta.csv';
time_in_ms = true;
tail_samples = 50;
step_detect_frac = 0.05;
% --------------------------

% --- carga datos ---
data = csvread(csvfile, 1, 0);
t_raw = data(:,1);
if time_in_ms
  t_raw = t_raw / 1000;
end
u_raw = data(:,2);
y_raw = data(:,3);

% --- Detectar los dos movimientos ---

% Suavizar la señal de posición para detectar cambios
y_smooth = y_raw;
if length(y_raw) > 5
  y_smooth = filter(ones(1,5)/5, 1, y_raw);  % Media móvil
end

% Encontrar índice del máximo (punto más lejos)
[max_dist, idx_max] = max(y_smooth);
printf('\n[DEBUG] Distancia máxima: %.2f cm en índice %d (t=%.2fs)\n', max_dist, idx_max, t_raw(idx_max));

% Encontrar primer cambio de PWM (inicio del alejamiento)
idx_start = find(u_raw > 50, 1, 'first');
printf('[DEBUG] Primer PWM>50 en índice %d (t=%.2fs)\n', idx_start, t_raw(idx_start));

% ========================================================================
% SEGMENTO 1: ALEJAMIENTO
% ========================================================================
% Desde el primer PWM hasta un poco después del máximo
idx_end_alej = idx_max + 50;  % Reducido a 50 muestras después del pico
if idx_end_alej > length(t_raw)
  idx_end_alej = length(t_raw);
end

% Verificar que realmente hay alejamiento
if y_raw(idx_end_alej) > y_raw(idx_start)
  t_alej = t_raw(idx_start:idx_end_alej);
  u_alej = u_raw(idx_start:idx_end_alej);
  y_alej = y_raw(idx_start:idx_end_alej);
  t_alej = t_alej - t_alej(1);

  printf('[DEBUG] Alejamiento: índices %d-%d, y: %.2f->%.2f cm\n', ...
         idx_start, idx_end_alej, y_raw(idx_start), y_raw(idx_end_alej));
else
  error('No se detectó alejamiento válido');
end

% ========================================================================
% SEGMENTO 2: ACERCAMIENTO
% ========================================================================

% Buscar desde el máximo hacia adelante
idx_start_acer = idx_max;
threshold_drop = 1.0;  % Debe bajar al menos 1 cm
for i = (idx_max+1):(length(y_raw)-10)
  % Verificar que en las próximas 10 muestras la tendencia es bajar
  future_samples = y_raw(i:min(i+10, length(y_raw)));
  if (y_raw(i) < y_raw(idx_max) - threshold_drop) && ...
     (mean(diff(future_samples)) < -0.1)  % Tendencia negativa
    idx_start_acer = i;
    break;
  end
end

printf('[DEBUG] Acercamiento detectado en índice %d (t=%.2fs, y=%.2f cm)\n', ...
       idx_start_acer, t_raw(idx_start_acer), y_raw(idx_start_acer));

% Tomar desde inicio del acercamiento hasta el final
t_acer = t_raw(idx_start_acer:end);
u_acer = u_raw(idx_start_acer:end);
y_acer = y_raw(idx_start_acer:end);

% Verificar que realmente se acerca
if y_acer(end) < y_acer(1) - 5  % Debe bajar al menos 5 cm
  t_acer = t_acer - t_acer(1);
  printf('[DEBUG] Acercamiento válido: y: %.2f->%.2f cm (cambio: %.2f cm)\n', ...
         y_acer(1), y_acer(end), y_acer(1)-y_acer(end));
else
  error('No se detectó acercamiento válido (cambio insuficiente)');
end

% ========================================================================
% FUNCIÓN PARA IDENTIFICAR UN MOVIMIENTO
% ========================================================================
function [K, tau, G, Fs] = identificar_movimiento(t, u, y, nombre)

  N = length(t);

  % Calcular frecuencia de muestreo
  dt = mean(diff(t));
  Fs = 1 / dt;

  % Promedios
  tail_samples = min(50, N);

  y_final = mean(y(N-tail_samples+1 : N));
  y_init  = mean(y(1 : min(tail_samples, N)));
  delta_y = abs(y_final - y_init);

  % Verificar que hubo movimiento significativo
  if delta_y < 1.0
    error('No se detectó movimiento significativo en %s (delta_y = %.2f cm)', nombre, delta_y);
  end

  % Para el escalón, asumimos PWM constante
  u_final = mean(u(N-tail_samples+1 : N));
  u_init  = mean(u(1 : min(tail_samples, N)));

  % Si no hay cambio en PWM, usar el valor promedio como amplitud
  if abs(u_final - u_init) < 1
    step_amp = mean(u);  % PWM promedio durante el movimiento
    printf('  [INFO] PWM constante detectado, usando valor promedio: %.1f\n', step_amp);
  else
    step_amp = u_final - u_init;
  end

  % Detectar inicio del movimiento (cambio significativo en posición)
  dy_threshold = 0.5;  % cm
  dy = abs(diff(y));
  idx_step = find(dy > dy_threshold, 1, 'first');
  if isempty(idx_step)
    idx_step = 1;
    printf('  [WARN] No se detectó cambio brusco, usando inicio del segmento\n');
  end

  t0 = t(idx_step);
  t_rel = t - t0;
  idx_after = find(t_rel >= 0);
  t_fit = t_rel(idx_after);

  y0 = mean(y(1:min(10, idx_step)));  % Baseline inicial
  y_rel = abs(y(idx_after) - y0);  % Magnitud del cambio

  % Estimación de K basada en el cambio total de posición
  K_est = delta_y / step_amp;

  % Optimización
  model_err = @(p) sum((y_rel - abs(p(1)) * step_amp .* (1 - exp(-t_fit ./ exp(p(2))))).^2);

  % Estimación inicial de tau
  % Buscar el tiempo para alcanzar 63% del cambio
  target_63 = y0 + 0.632 * delta_y;

  % Buscar en la señal
  idx63 = -1;
  for i = 1:length(y)
    if abs(y(i) - y0) >= 0.632 * delta_y
      idx63 = i;
      break;
    end
  end

  if idx63 > 0 && idx63 <= length(t_fit)
    tau0 = t_fit(idx63);
    if tau0 <= 0 || tau0 > 50  % Límite razonable
      tau0 = 2.0;
      printf('  [WARN] tau0 fuera de rango, usando 2.0s\n');
    end
  else
    tau0 = 2.0;
    printf('  [WARN] No se encontró punto 63%%, usando tau0=2.0s\n');
  end

  printf('  [DEBUG] tau0 inicial estimado: %.2f s\n', tau0);

  p0 = [K_est, log(tau0)];

  % Límites más estrictos para evitar resultados absurdos
  lb = [0.01, log(0.5)];
  ub = [10.0, log(20.0)];

  options = optimset('Display','off', 'MaxIter', 2000, 'TolX', 1e-8);

  % Usar fmincon si está disponible, sino fminsearch con penalización
  try
    pkg load optim
    popt = fmincon(model_err, p0, [], [], [], [], lb, ub, [], options);
  catch
    % Fallback: fminsearch con penalización
    penalized_err = @(p) model_err(p) + ...
                         1e6 * (p(1) < lb(1) || p(1) > ub(1) || ...
                               p(2) < lb(2) || p(2) > ub(2));
    popt = fminsearch(penalized_err, p0, options);
  end

  K = abs(popt(1));
  tau = exp(popt(2));

  % Verificación de sanidad
  if K > 5 || tau > 15
    printf('  [WARN] Parámetros sospechosos (K=%.2f, tau=%.2f), usando valores conservadores\n', K, tau);
    K = min(K, 0.5);
    tau = min(tau, 5.0);
  end

  G = tf(K, [tau 1]);

  % Mostrar resultados
  printf('\n=== %s ===\n', nombre);
  printf('Muestras: %d | Duración: %.2f s | Fs: %.2f Hz\n', N, t(end)-t(1), Fs);
  printf('PWM usado: %.2f | Cambio en posición: %.2f cm\n', step_amp, delta_y);
  printf('K = %.6g cm/PWM | tau = %.4g s | Polo = %.4g\n', K, tau, -1/tau);
  printf('Tiempos: 63%%=%.2fs | 95%%=%.2fs | 98%%=%.2fs\n', tau, 3*tau, 4*tau);

end

% ========================================================================
% IDENTIFICAR AMBOS MOVIMIENTOS
% ========================================================================

[K_alej, tau_alej, G_alej, Fs_alej] = identificar_movimiento(t_alej, u_alej, y_alej, 'ALEJAMIENTO');
[K_acer, tau_acer, G_acer, Fs_acer] = identificar_movimiento(t_acer, u_acer, y_acer, 'ACERCAMIENTO');

% ========================================================================
% COMPARACIÓN
% ========================================================================
printf('\n=== COMPARACIÓN ===\n');
printf('              Alejamiento    Acercamiento\n');
printf('K (cm/PWM):   %.6g        %.6g\n', K_alej, K_acer);
printf('tau (s):      %.4g          %.4g\n', tau_alej, tau_acer);
printf('Diferencia K: %.2f%%\n', abs(K_alej - K_acer) / K_alej * 100);
printf('Diferencia tau: %.2f%%\n', abs(tau_alej - tau_acer) / tau_alej * 100);

% Modelo promedio
K_avg = (abs(K_alej) + abs(K_acer)) / 2;
tau_avg = (tau_alej + tau_acer) / 2;
G_avg = tf(K_avg, [tau_avg 1]);

printf('\n=== MODELO PROMEDIO ===\n');
printf('K = %.6g cm/PWM\n', K_avg);
printf('tau = %.4g s\n', tau_avg);
disp('G(s):'); G_avg

% ========================================================================
% GRÁFICAS
% ========================================================================

% Figura 1: Datos completos
figure(1);
subplot(2,1,1);
plot(t_raw, u_raw, 'b-', 'LineWidth', 1.5);
hold on;
xlabel('Tiempo (s)');
ylabel('PWM');
title('Señal de entrada');
grid on;

subplot(2,1,2);
plot(t_raw, y_raw, 'b-', 'LineWidth', 1.5);
hold on;
xlabel('Tiempo (s)');
ylabel('Distancia (cm)');
title('Posición');
grid on;

% Figura 2: Comparación de modelos
figure(2);
subplot(1,2,1);
step(G_alej * 255);
title('Alejamiento');
grid on;

subplot(1,2,2);
step(G_acer * 255);
title('Acercamiento');
grid on;

% Figura 3: Polos
figure(3);
subplot(1,3,1);
pzmap(G_alej);
legend('off');
title('Polos - Alejamiento');
grid on;

subplot(1,3,2);
pzmap(G_acer);
legend('off');
title('Polos - Acercamiento');
grid on;

subplot(1,3,3);
pzmap(G_avg);
legend('off');
title('Polos - Promedio');
grid on;

