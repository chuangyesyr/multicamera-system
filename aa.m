ss{1} = 'r-';
ss{2} = 'k-.';
ss{3} = 'b--';

handles.Time = 4000;

figure;                                    % Energy Left
for i = 1:3
    plot(1:handles.Time, CC(i, 1:handles.Time), ss{i}, 'MarkerSize', 3);
    hold on;
end
s_s{1} = 'No Energy Consideration';
s_s{2} = 'Ec considered';
s_s{3} = 'Er considered';
xlabel('Time');
ylabel('Energy Left');
legend(s_s, 1-i);
hold on;
a = 1896;
plot(a, 0.8, 'r*');
text(a-600, 0.7, 'Ec = 0.8');
text(a-600, 0.5, '(1896, 0.8)');