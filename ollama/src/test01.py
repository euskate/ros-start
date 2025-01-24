import ollama

def ask_question(question):
    response = ollama.chat(
        model="llava:7b",
        messages=[
            {
                'role':'user',
                'content': question
            }
        ]
    )
    return response

response = ask_question("안녕 반가워")
print(response.__dict__)
print(response.message.content)
