import PyPDF2

# 打开PDF文件
with open('rules.pdf', 'rb') as pdf_file:
    # 创建PDF阅读器对象
    pdf_reader = PyPDF2.PdfReader(pdf_file)
    
    # 初始化文本变量
    text = ''
    
    # 遍历所有页面并提取文本
    for page_num in range(len(pdf_reader.pages)):
        page = pdf_reader.pages[page_num]
        text += page.extract_text() + '\n'

# --- 新增：将提取的文本保存到txt文件 ---
with open('output.txt', 'w', encoding='utf-8') as txt_file:
    txt_file.write(text)

print("文本已成功提取并保存到 output.txt")