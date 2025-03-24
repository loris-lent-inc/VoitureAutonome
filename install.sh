echo "Creation de l'environnement"
python3 -m venv --system-site-packages venv
source venv/bin/activate

echo "Mise a jour PIP"
pip install --upgrade pip

echo "installation des dependances"
if [ -f "requirements.txt" ]; then
	pip install -r requirements.txt
else 
	echo "Fichier requirements.txt manquant"
	exit 1
fi