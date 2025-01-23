CREATE TABLE gas_data (
    id INT AUTO_INCREMENT PRIMARY KEY,
    ammonia FLOAT,
    carbon_dioxide FLOAT,
    methane FLOAT,
    nitrous_oxide FLOAT,
    hydrogen_sulfide FLOAT,
    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
);

