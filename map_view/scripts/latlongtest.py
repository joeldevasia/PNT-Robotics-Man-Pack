import utm

latitude_longitude = utm.to_latlon(easting=572132.39, northing=3457839.01, zone_number=43, northern=False)
print(latitude_longitude[0], latitude_longitude[1])