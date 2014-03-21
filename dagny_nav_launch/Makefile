all: dagny_002_path.png


dagny_002_path.png: dagny_002.mprim reachability.py
	./reachability.py dagny_002.mprim -o dagny_002 -p -r 80 -i 2

dagny_002.mprim: dagny_002.yaml mprim_gen.py
	./mprim_gen.py -r 0.02 -m 0.7 -y dagny_002.yaml -o dagny_002.mprim 
