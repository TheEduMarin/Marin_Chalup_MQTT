import json
import boto3


iot_data = boto3.client('iot-data', region_name='us-east-1') 

THING_NAME = 'ESP32'  

def lambda_handler(event, context):
    intent_name = event['request']['intent']['name']
    

    if intent_name == 'GetDistanceIntent':
        try:

            response = iot_data.get_thing_shadow(thingName=THING_NAME)
            payload = json.loads(response['payload'].read().decode('utf-8'))
            
            distance = payload['state']['reported'].get('distance', None)
            
            if distance is not None:
                return build_response(f"La distancia medida es de {distance} centímetros.")
            else:
                return build_response("Aún no hay una distancia reportada por el dispositivo.")
        except Exception as e:
            print(f"Error al obtener la distancia: {e}")
            return build_response("Error al obtener la distancia.")
    

    elif intent_name == 'MoveServoIntent':
        action = event['request']['intent']['slots']['Action']['value']
        if action not in ['abrir', 'cerrar']:
            return build_response("Solo puedo abrir o cerrar el servomotor.")
        

        desired_state = {'servoState': action}
        update_payload = {
            'state': {
                'desired': desired_state
            }
        }
        
        try:
            iot_data.update_thing_shadow(
                thingName=THING_NAME,
                payload=json.dumps(update_payload)
            )
            return build_response(f"He enviado la orden para {action} el servomotor.")
        except Exception as e:
            print(f"Error al mover el servo: {e}")
            return build_response("No pude enviar la orden al servomotor.")
    
    return build_response("No entendí tu solicitud.")

def build_response(text):
    return {
        "version": "1.0",
        "response": {
            "outputSpeech": {
                "type": "PlainText",
                "text": text
            },
            "shouldEndSession": True
        }
    }
