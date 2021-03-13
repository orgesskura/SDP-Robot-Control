# Requires pip install pyrebase

# Import python wrapper for firebase API: pyrebase
import pyrebase

def UpdateDatabase():
	
	# Connect to database

	config = {
	  "apiKey": "AIzaSyAQaLZ5NMpNg48kV5ey4lRBpC7FT4cYc98",
	  "authDomain": "sdp-application.firebaseapp.com",
	  "databaseURL": "https://sdp-application-default-rtdb.firebaseio.com",
	  "storageBucket": "sdp-application.appspot.com"
	}
	
	firebase = pyrebase.initialize_app(config)
	
	# Get a reference to the auth service
	auth = firebase.auth()
	
	# Get a reference to the database service
	db = firebase.database()
	
	# get robot details (temporarily hardcoded to test)
	latitude = 56.647299
	longitude = -2.762054
	battery = 43
	
	# Update the database
	db.child("Test").update({"Battery": 24})
	db.child("Test").child("Location").update({"Latitude": latitude})
	db.child("Test").child("Location").update({"Longitude": longitude})
