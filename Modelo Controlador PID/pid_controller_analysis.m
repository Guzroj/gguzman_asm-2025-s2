% pid_controller_analysis.m
% Diagrama de polos y ceros del controlador PID
% C(s) = Kp + Ki/s + Kd*s

clc; clear; close all;

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

% OPCIÓN 1: Usando tf con tiempo continuo explícito
num_pid = [Kd, Kp, Ki];   % Kd*s^2 + Kp*s + Ki
den_pid = [1, 0];          % s
C = tf(num_pid, den_pid, 'TimeUnit', 'seconds');

% OPCIÓN 2 (ALTERNATIVA): Usando variable simbólica 's'
% s = tf('s');
% C = Kp + Ki/s + Kd*s;

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

% Figura 1: Diagrama de polos y ceros (MANUAL - sin pzmap para evitar grid discreto)
figure(1);
clf;

% Calcular límites
xlim_val = max(abs([real(polos); real(ceros); 1])) * 1.5;
ylim_val = max(abs([imag(polos); imag(ceros); 1])) * 1.5;

% Crear figura
hold on;
grid on;
axis equal;

% Dibujar ejes
plot([-xlim_val xlim_val], [0 0], 'k-', 'LineWidth', 1);  % Eje real
plot([0 0], [-ylim_val ylim_val], 'k-', 'LineWidth', 1);  % Eje imaginario

% Dibujar polos (x azul)
if ~isempty(polos)
  plot(real(polos), imag(polos), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
end

% Dibujar ceros (o rojo)
if ~isempty(ceros)
  plot(real(ceros), imag(ceros), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
end

% Configurar ejes y etiquetas
xlim([-xlim_val, xlim_val]);
ylim([-ylim_val, ylim_val]);
xlabel('Eje Real (segundos^{-1})');
ylabel('Eje Imaginario (segundos^{-1})');
title(sprintf('Diagrama de Polos y Ceros del Controlador PID (Tiempo Continuo)\nKp=%.2f, Ki=%.2f, Kd=%.2f', Kp, Ki, Kd));

% Leyenda
if ~isempty(polos) && ~isempty(ceros)
  legend('Polos', 'Ceros', 'Location', 'best');
elseif ~isempty(polos)
  legend('Polos', 'Location', 'best');
elseif ~isempty(ceros)
  legend('Ceros', 'Location', 'best');
end

hold off;


fprintf('\n¡Análisis completado!\n');




