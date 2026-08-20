# 部署指南

## 云服务部署说明

本项目支持部署到腾讯云 CloudBase，以下是详细部署步骤：

### 方式一：使用 CloudBase CLI 部署

#### 1. 安装 CloudBase CLI

```bash
npm install -g @cloudbase/cli
```

#### 2. 登录 CloudBase

```bash
cloudbase login
```

#### 3. 创建云开发环境

登录腾讯云控制台 (https://console.cloud.tencent.com/tcb)，创建新的云开发环境。

#### 4. 配置部署参数

编辑 `cloudbaserc.json` 文件，填写您的环境信息：

```json
{
  "envId": "your-env-id",
  "region": "ap-shanghai"
}
```

#### 5. 部署项目

```bash
cloudbase deploy
```

#### 6. 访问应用

部署完成后，CloudBase 会提供一个访问 URL。

### 方式二：使用 CloudBase Web 控制台部署

1. 登录腾讯云控制台
2. 进入云开发控制台
3. 创建或选择环境
4. 点击"静态网站托管"
5. 上传构建产物 (dist 目录)
6. 配置访问域名

### 方式三：部署到其他云服务

#### Vercel 部署

1. 将代码推送到 GitHub
2. 在 Vercel 导入项目
3. Vercel 会自动构建和部署

#### Netlify 部署

1. 将代码推送到 GitHub
2. 在 Netlify 导入项目
3. 配置构建设置：
   - Build command: `npm run build`
   - Publish directory: `dist`

## 部署前准备

### 1. 构建项目

```bash
npm run build
```

### 2. 检查构建产物

确保 `dist` 目录包含以下文件：
- index.html
- assets/ 目录
- 所有必要的静态资源

### 3. 配置文档路径

确保所有文档路径正确指向原始资料文件夹：

```typescript
// 在 documents.ts 中
path: '../1.用户手册/xxx.pdf'
```

## 部署后配置

### 1. 配置文档访问

由于文档文件位于原始文件夹，需要：

**方案 A：复制文档到项目**
```bash
# 复制所有文档到项目的 public 目录
cp -r ../资料包/* public/
```

**方案 B：配置代理**
在部署服务器上配置反向代理，将文档请求转发到实际位置。

**方案 C：使用 CDN**
将文档上传到 CDN，修改文档路径为 CDN 地址。

### 2. 配置域名

在云服务控制台配置自定义域名。

### 3. 配置 HTTPS

大多数云服务支持一键开启 HTTPS。

## 故障排查

### 文档无法访问

检查以下几点：
1. 文档路径是否正确
2. 文档是否存在
3. 服务器是否配置了正确的 MIME 类型

### 搜索功能异常

1. 检查浏览器控制台是否有错误
2. 确保 documents.ts 数据正确
3. 清除浏览器缓存重试

### 部署失败

1. 检查网络连接
2. 确认构建命令正确
3. 查看 CloudBase 部署日志

## 维护更新

### 更新文档

1. 更新 `src/data/documents.ts` 中的文档信息
2. 重新构建：`npm run build`
3. 重新部署：`cloudbase deploy`

### 更新问答库

1. 编辑 `src/data/qaData.ts`
2. 重新构建和部署

### 功能更新

1. 修改代码
2. 测试功能
3. 构建和部署

## 性能优化

### 1. 启用 CDN

为静态资源配置 CDN 加速。

### 2. 启用缓存

配置适当的缓存策略。

### 3. 代码分割

Vite 默认会进行代码分割，无需额外配置。

## 安全建议

1. 不要在前端代码中暴露敏感信息
2. 使用 HTTPS
3. 定期更新依赖包
4. 配置适当的安全头

## 成本说明

CloudBase 免费额度：
- 静态网站托管：免费
- 流量：每月 5GB
- 存储空间：5GB

超出免费额度会产生费用，具体请查看腾讯云价格表。
