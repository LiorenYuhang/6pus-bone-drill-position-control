# WHEELTEC H30 INS Knowledge Base

WHEELTEC H30 惯导产品资料在线问答系统

## 项目简介

这是一个基于 React + Vite 构建的在线知识库系统，用于管理和查询 WHEELTEC H30 惯导模块的相关文档、代码和资料。

## 功能特性

### 1. 文档浏览
- 分类浏览所有文档（用户手册、ROS SDK、软件工具、芯片手册、机械模型等）
- 支持 PDF、代码文件、压缩包等多种格式
- 文件描述和类型标签展示

### 2. 智能搜索
- 全文搜索功能，支持关键词匹配
- 搜索结果按相关性排序
- 搜索历史记录

### 3. 智能问答
- 基于预设问答库的智能问答系统
- 相似问题推荐
- 常见问题快速解答

### 4. 收藏功能
- 收藏重要文档
- 本地存储，无需登录
- 一键管理收藏列表

### 5. 代码查看
- 在线查看代码文件
- 语法高亮显示
- 支持多种编程语言（Python、C/C++、JavaScript、TypeScript等）

### 6. 中英文双语
- 界面支持中英文切换
- 即时语言切换

### 7. 响应式设计
- 适配桌面和移动设备
- 深色模式支持

## 技术栈

- **前端框架**: React 18 + TypeScript
- **构建工具**: Vite 5
- **UI 组件**: TailwindCSS + 自定义组件
- **图标库**: Lucide React
- **代码高亮**: react-syntax-highlighter
- **国际化**: i18next + react-i18next
- **状态管理**: React Hooks (useState, useEffect)
- **本地存储**: LocalStorage

## 项目结构

```
h30-ins-knowledge-base/
├── src/
│   ├── components/          # React组件
│   │   ├── Header.tsx      # 顶部导航栏
│   │   ├── Sidebar.tsx     # 左侧分类导航
│   │   ├── DocumentCard.tsx # 文档卡片
│   │   ├── SearchResults.tsx # 搜索结果
│   │   ├── Favorites.tsx   # 收藏夹
│   │   ├── History.tsx     # 搜索历史
│   │   ├── QASection.tsx   # 问答区
│   │   └── CodeViewer.tsx  # 代码查看器
│   ├── data/               # 数据文件
│   │   ├── documents.ts    # 文档元数据
│   │   └── qaData.ts       # 问答数据
│   ├── hooks/              # 自定义Hooks
│   │   └── useLocalStorage.ts
│   ├── utils/              # 工具函数
│   │   ├── search.ts       # 搜索功能
│   │   └── qa.ts          # 问答功能
│   ├── types.ts           # TypeScript类型定义
│   ├── i18n.ts            # 国际化配置
│   ├── App.tsx            # 主应用组件
│   └── main.tsx           # 入口文件
├── public/                # 静态资源
└── ...配置文件
```

## 快速开始

### 安装依赖

```bash
npm install
```

### 启动开发服务器

```bash
npm run dev
```

访问 http://localhost:5173 查看应用

### 构建生产版本

```bash
npm run build
```

## 文档数据说明

文档数据存储在 `src/data/documents.ts` 文件中，包含所有文档的元信息：
- 文档名称
- 分类（用户手册、ROS SDK等）
- 文件类型（PDF、代码、压缩包）
- 文件路径
- 描述信息

如需添加新文档，请在 `documents.ts` 中添加相应的文档对象。

## 问答数据说明

问答数据存储在 `src/data/qaData.ts` 文件中，包含常见问题和答案。

如需添加新问题，请在 `qaData.ts` 中添加问答对。

## 部署说明

### 本地部署

1. 构建项目：
```bash
npm run build
```

2. 将 `dist` 目录部署到 Web 服务器

### 云服务部署

项目可以部署到以下云服务：
- CloudBase（腾讯云）
- Vercel
- Netlify
- GitHub Pages

具体部署步骤请参考各平台文档。

## 使用说明

### 搜索文档
1. 在顶部搜索框输入关键词
2. 点击搜索按钮或按回车
3. 查看搜索结果

### 浏览分类
1. 点击左侧分类菜单
2. 查看该分类下的所有文档

### 收藏文档
1. 找到要收藏的文档
2. 点击文档卡片右上角的爱心图标
3. 在收藏夹中查看已收藏文档

### 查看代码
1. 找到代码类型的文档
2. 点击"查看代码"按钮
3. 在代码查看器中查看代码

### 智能问答
1. 在问答区域输入问题
2. 点击提交按钮
3. 查看系统推荐的答案

## 浏览器支持

- Chrome (推荐)
- Firefox
- Safari
- Edge

## 许可证

© 2025 WHEELTEC. All rights reserved.

## 联系方式

如有问题，请查阅产品文档或联系技术支持。
