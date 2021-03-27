# Requires pip install pyrebase

# Import python wrapper for firebase API: pyrebase
import pyrebase

import utils

offset = utils.longlat_position(-3.161150, 55.952237)

def updateDatabase(long_lat_pos, battery, fullness):
	global offset
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
	latitude = long_lat_pos.latitude + offset.latitude
	longitude = long_lat_pos.longitude + offset.longitude
	
	# Update the database
	try:
		db.child("Test").update({"Battery": battery})
		db.child("Test").child("Location").update({"Latitude": latitude})
		db.child("Test").child("Location").update({"Longitude": longitude})
		db.child("Test").update({"Fullness": fullness})
	except Exception as e:
		print(e)
		
def getTargetCoordinatesFromDatabase():
	global offset
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
	
	targetLatitude = float(db.child("Test").child("TargetLatitude").get().val()) - offset.latitude
	targetLongitude = float(db.child("Test").child("TargetLongitude").get().val()) - offset.longitude

	targ = utils.longlat_position(targetLongitude, targetLatitude)
	return targ
