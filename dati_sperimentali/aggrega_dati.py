import glob
import csv
from collections import defaultdict
import statistics

file_csv = sorted(glob.glob('/home/tommaso/rover_ws/log_missione/*.csv'))
print(f'Trovati {len(file_csv)} file CSV (run)\n')

righe_per_waypoint = defaultdict(list)
tutte_le_righe = []

for f in file_csv:
    with open(f) as fh:
        lettore = csv.DictReader(fh)
        for riga in lettore:
            tutte_le_righe.append(riga)
            righe_per_waypoint[riga['indice_waypoint']].append(riga)

totale_waypoint = len(tutte_le_righe)
successi = sum(1 for r in tutte_le_righe if r['esito'] == 'successo')
print(f'Waypoint totali: {totale_waypoint}')
print(f'Successi: {successi} ({100*successi/totale_waypoint:.1f}%)')
print(f'Foto totali salvate: {sum(int(r["foto_scattate"]) for r in tutte_le_righe)}')
print(f'Nuvole salvate: {sum(1 for r in tutte_le_righe if r["nuvola_salvata"]=="True")}/{totale_waypoint}')
print()

print(f'{"WP":<4}{"Tempo medio (s)":<18}{"Dev.std":<10}{"Recovery medi":<15}{"Recovery max"}')
for wp in sorted(righe_per_waypoint.keys(), key=int):
    righe = righe_per_waypoint[wp]
    tempi = [float(r['tempo_navigazione_s']) for r in righe]
    recoveries = [int(r['numero_recoveries']) for r in righe]
    media_t = statistics.mean(tempi)
    std_t = statistics.stdev(tempi) if len(tempi) > 1 else 0.0
    media_r = statistics.mean(recoveries)
    print(f'{wp:<4}{media_t:<18.1f}{std_t:<10.2f}{media_r:<15.2f}{max(recoveries)}')

print()
tempi_missione = []
for f in file_csv:
    with open(f) as fh:
        lettore = csv.DictReader(fh)
        tempo_tot = sum(float(r['tempo_navigazione_s']) for r in lettore)
        tempi_missione.append(tempo_tot)

print(f'Tempo totale medio per missione completa: {statistics.mean(tempi_missione):.1f}s '
      f'(dev.std {statistics.stdev(tempi_missione):.1f}s)')
