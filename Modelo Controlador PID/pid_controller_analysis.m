% pid_controller_analysis.m
% Diagrama de polos y ceros del controlador PID
% C(s) = Kp + Ki/s + Kd*s

clc; clear; close all;

pkg load control

% ========================================
% PARÁMETROS DEL CONTROLADOR PID
% ========================================
Kp = 5.0;   % Ganancia proporcional
Ki = 0.8;   % Ganancia integral
Kd = 0.3;   % Ganancia derivativa

fprintf('\n========================================\n');
fprintf('ANÁLISIS DEL CONTROLADOR PID\n');
fprintf('========================================\n\n');

fprintf('Parámetros del controlador:\n');
fprintf('  Kp = %.4g\n', Kp);
fprintf('  Ki = %.4g\n', Ki);
fprintf('  Kd = %.4g\n', Kd);

% ========================================
% FUNCIÓN DE TRANSFERENCIA DEL PID
% ========================================
% C(s) = Kp + Ki/s + Kd*s
% C(s) = (Kd*s^2 + Kp*s + Ki) / s

num_pid = [Kd, Kp, Ki];   % Kd*s^2 + Kp*s + Ki
den_pid = [1, 0];          % s

C = tf(num_pid, den_pid);

fprintf('\nFunción de transferencia del controlador PID:\n');
fprintf('C(s) = Kp + Ki/s + Kd*s\n');
fprintf('     = (Kd*s^2 + Kp*s + Ki) / s\n\n');

disp('C(s):');
C

% ========================================
% POLOS Y CEROS
% ========================================
polos = pole(C);
ceros = zero(C);

fprintf('\n========================================\n');
fprintf('POLOS DEL CONTROLADOR\n');
fprintf('========================================\n');
if isempty(polos)
  fprintf('No hay polos finitos\n');
else
  for i = 1:length(polos)
    if imag(polos(i)) ~= 0
      fprintf('Polo %d: s = %.4f ± %.4fj\n', i, real(polos(i)), abs(imag(polos(i))));
    else
      fprintf('Polo %d: s = %.4f\n', i, polos(i));
    end
  end
end

fprintf('\n========================================\n');
fprintf('CEROS DEL CONTROLADOR\n');
fprintf('========================================\n');
if isempty(ceros)
  fprintf('No hay ceros finitos\n');
else
  for i = 1:length(ceros)
    if imag(ceros(i)) ~= 0
      fprintf('Cero %d: s = %.4f ± %.4fj\n', i, real(ceros(i)), abs(imag(ceros(i))));
    else
      fprintf('Cero %d: s = %.4f\n', i, ceros(i));
    end
  end
end

% ========================================
% INTERPRETACIÓN
% ========================================
fprintf('\n========================================\n');
fprintf('INTERPRETACIÓN\n');
fprintf('========================================\n');

% El PID tiene un polo en el origen (por el término Ki/s)
fprintf('\n1. POLO EN EL ORIGEN (s = 0):\n');
fprintf('   - Introducido por el término integral Ki/s\n');
fprintf('   - Mejora el seguimiento y elimina error estacionario\n');
fprintf('   - Puede causar inestabilidad si Ki es muy grande\n');

% Calcular los ceros (raíces del numerador)
if ~isempty(ceros)
  fprintf('\n2. CEROS DEL CONTROLADOR:\n');
  fprintf('   Ubicación: s = %.4f', ceros(1));
  if length(ceros) > 1
    fprintf(', s = %.4f', ceros(2));
  end
  fprintf('\n');

  % Verificar si los ceros están en el semiplano izquierdo (fase mínima)
  if all(real(ceros) < 0)
    fprintf('   Todos los ceros en semiplano izquierdo (fase mínima)\n');
    fprintf('   El controlador no introduce retardos de fase excesivos\n');
  else
    fprintf('   Algún cero en semiplano derecho (fase no mínima)\n');
    fprintf('   Puede introducir retardos de fase no deseados\n');
  end

  % Verificar relación entre ceros
  if length(ceros) == 2
    discriminante = Kp^2 - 4*Kd*Ki;
    if discriminante < 0
      fprintf('   - Ceros complejos conjugados\n');
      fprintf('   - Frecuencia de los ceros: %.4f rad/s\n', abs(imag(ceros(1))));
    else
      fprintf('   - Ceros reales\n');
    end
  end
end



fprintf('\n========================================\n');

% ========================================
% GRÁFICAS
% ========================================

% Figura 1: Diagrama de polos y ceros
figure(1);
pzmap(C);
legend('off');
title(sprintf('Diagrama de Polos y Ceros del Controlador PID\nKp=%.2f, Ki=%.2f, Kd=%.2f', Kp, Ki, Kd));
grid on;
axis equal;

% Mejorar visualización
xlim_val = max(abs([real(polos); real(ceros)])) * 1.5;
if xlim_val == 0
  xlim_val = 10;
end
ylim_val = max(abs([imag(polos); imag(ceros)])) * 1.5;
if ylim_val == 0
  ylim_val = 10;
end
xlim([-xlim_val, xlim_val]);
ylim([-ylim_val, ylim_val]);

% Agregar líneas de referencia
hold on;
plot([0 0], ylim, 'k:', 'LineWidth', 0.5);  % Eje imaginario
plot(xlim, [0 0], 'k:', 'LineWidth', 0.5);  % Eje real




