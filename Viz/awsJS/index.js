AWS.config.update({
    region: "us-east-2",
    endpoint: 'http://dynamodb.us-east-2.amazonaws.com',
    // accessKeyID should never be revealed
    accessKeyId: "AKIA52GYYF6ZOANANBFH",
    // secretAccessKey should never be revealed
    secretAccessKey: "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"
});
  
var docClient = new AWS.DynamoDB.DocumentClient();
  
// Query for a single scan. Need to know the scan name for this to work.
function queryData(input) {
    document.getElementById('textarea').innerHTML += "Querying for scans." + "\n";

    var params = {
        TableName : "SENTINEL",
        KeyConditionExpression: "#yr = :yyyy",
        ExpressionAttributeNames:{
            "#yr": "Name"
        },
        ExpressionAttributeValues: {
            ":yyyy":input
        }
    };

    docClient.query(params, function(err, data) {
        if (err) {
            document.getElementById('textarea').innerHTML += "Unable to query. Error: " + "\n" + JSON.stringify(err, undefined, 2);
        } else {
            document.getElementById('textarea').innerHTML += "Querying for scans: " + "\n";
            listOfNames = data.Items[0]['list']
            for (i = 0; i < listOfNames.length; i++){
                document.getElementById('textarea').innerHTML += listOfNames[i] + "\n";
            }
            return JSON.stringify(data, undefined, 2);
        }
    });
}

// Sample scan data function provided by AWS
// function scanData() {
//     document.getElementById('textarea').innerHTML += "Scanning Movies table." + "\n";

//     var params = {
//         TableName: "SENTINEL",
//         ProjectionExpression: "#yr",
//         ExpressionAttributeNames: {
//             "#yr": "Name",
//         },
//     };

//     document.getElementById('textarea').innerHTML += "Scan succeeded. " + "\n";
//         data.Items.forEach(function(movie) {
//             document.getElementById('textarea').innerHTML += string(movie) + "\n";
//         });

//         // Continue scanning if we have more movies (per scan 1MB limitation)
//         document.getElementById('textarea').innerHTML += "Scanning for more..." + "\n";
//         params.ExclusiveStartKey = data.LastEvaluatedKey;
//         docClient.scan(params, onScan);            
//     }

    function scanData() {
        document.getElementById('textarea').innerHTML += "Scanning Movies table." + "\n";
    
        var params = {
            TableName: "SENTINEL",
            ProjectionExpression: "#yr",
            ExpressionAttributeNames: {
                "#yr": "year",
            },
            ExpressionAttributeValues: {
                ":start_yr": 1950,
                ":end_yr": 1959
            }
        };
    
        docClient.scan(params, onScan);
    
        function onScan(err, data) {
            if (err) {
                document.getElementById('textarea').innerHTML += "Unable to scan the table: " + "\n" + JSON.stringify(err, undefined, 2);
            } else {
                // Print all the movies
                document.getElementById('textarea').innerHTML += "Scan succeeded. " + "\n";
                data.Items.forEach(function(movie) {
                    document.getElementById('textarea').innerHTML += movie.Name + "\n";
                });
    
                // Continue scanning if we have more movies (per scan 1MB limitation)
                document.getElementById('textarea').innerHTML += "Scanning for more..." + "\n";
                params.ExclusiveStartKey = data.LastEvaluatedKey;
                docClient.scan(params, onScan);            
            }
        }
    }

// Grab all the scans from AWS and populate them in a dropdown list
function listPopulate() {

}



